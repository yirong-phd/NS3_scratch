#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <cmath>
#include <iostream>
#include <fstream>
using namespace ns3;

//define all the global variables:
int Npkt_1 = 0; int N_pkt_1_ob = 0; int Npkt_1_rx = 0;
int Npkt_2 = 0; int N_pkt_2_ob = 0; int Npkt_2_rx = 0;

double prev_on = 0.0; double prev_ia = 0.0;
double ia = 0.0; double on = 0.0;
int N_on = 0; int N_ia = 0;

bool L1 = false; bool L2 = false; // false == OFF; true == ON;

double est_pr2, real_pr2;

// The guess is that the relative time-stamp for wireshark and NS3 tracer is different, but based on the
// experiment, the time-duration of the ON/OFF process and the total number of packets are the same.

// The next challenge is to replace OnOffApplication with self-written packet socket implementations, as it
// provide random on and off intervals for the traffic. We solve this problem by self-writing a packet generation function
// and a packet reception function for socket callback.

void ReceivePacket (Ptr<Socket> socket)
{
	Address from;
	Ptr<Packet> p;
  while ((p = socket->RecvFrom(from)))
    {
			if (p->GetSize() > 0) {
      	//NS_LOG_UNCOND ("Received one packet!");
			}
    }
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             double sim_time, Ptr<ExponentialRandomVariable> pktInterval)
{
  if (sim_time > 0)
    {
      //double t_tx = Simulator::Now().GetSeconds();
			Ptr<Packet> data_packet = Create<Packet> (pktSize);
			socket->Send(data_packet);
      //double t_rx = Simulator::Now().GetSeconds();
      double Interval_t = pktInterval->GetValue();
      Simulator::Schedule (Seconds(Interval_t), &GenerateTraffic,
                           socket, pktSize, sim_time-Interval_t, pktInterval);
    }
  else
    {
      socket->Close();
    }
}

// Tracer callbacks:
void PhyTx1(Ptr<const Packet> p, double txPowerW) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyTx(B->A) at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    Npkt_1 ++;
    if(L2 == false){
      // if link 2 is off, then this packet is observable.
      N_pkt_1_ob ++;
    }
    L1 = true;
    if(prev_ia != 0){
      ia += (Simulator::Now().GetSeconds() - prev_ia);
      N_ia += 1;

    }

    // update the time-stamp for the start of current pkt Tx
    prev_on = Simulator::Now ().GetSeconds ();
  }
}

void PhyTx2(Ptr<const Packet> p, double txPowerW){
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyTx(D->C) at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    Npkt_2 ++;
    if(L1 == false){
      // if link 1 is off, then this packet is observable.
      N_pkt_2_ob ++;
    }
    L2 = true;
  }
}

void PhyRx1(Ptr<const Packet> p) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyEnd(B->A) at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    Npkt_1_rx ++;

    L1 = false;
    on += (Simulator::Now().GetSeconds() - prev_on);
    N_on += 1;
    // update the time-stamp for the start of current pkt Tx
    prev_ia = Simulator::Now ().GetSeconds ();

  }
}

void PhyRx2(Ptr<const Packet> p) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyEnd(D->C) at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    Npkt_2_rx ++;

    L2 = false;
  }

}

int main (int argc, char *argv[])
{

  uint32_t packetSize = 2000; // bytes
  //uint32_t numPackets = 10000; // a sufficient large enough number
  double mean = 0.002;
	double bound = 0.1;
	Ptr<ExponentialRandomVariable> interval = CreateObject<ExponentialRandomVariable> ();
	interval->SetAttribute ("Mean", DoubleValue (mean));
	interval->SetAttribute ("Bound", DoubleValue (bound));


  double sim_time = 5.0;
  uint16_t prop_loss = 1;   //by default the range prop loss model (ideal case)
  uint16_t outputmode = 1;  //mode 1 is more reader friendly, while mode 2 is for log

  std::string nA_txRate = "OfdmRate6Mbps"; // node A transmission rate
  std::string nB_txRate = "OfdmRate6Mbps"; // node B transmission rate
  std::string nC_txRate = "OfdmRate6Mbps"; // node C transmission rate
  std::string nD_txRate = "OfdmRate6Mbps"; // node D transmission rate
  std::string nE_txRate = "OfdmRate6Mbps"; // node E transmission rate

  CommandLine cmd;
  cmd.AddValue ("sim_time", "simulation time in seconds", sim_time);
  cmd.AddValue ("prop_loss", "select the propagation loss model: 1 for RangePropagationLossModel; 2 for Log dist model", prop_loss);
  cmd.AddValue ("outputmode", "selecte the output format: 1 reader friendly 2 for matlab", outputmode);
	cmd.AddValue ("ia_mean", "the mean value of the exponentially distributed pkt inter-arrival times", mean);
  cmd.Parse(argc, argv);

  NodeContainer nodes;
  nodes.Create (5);
  Ptr<Node> nA = nodes.Get (0);
  Ptr<Node> nB = nodes.Get (1);
  Ptr<Node> nC = nodes.Get (2);
  Ptr<Node> nD = nodes.Get (3);
  Ptr<Node> nE = nodes.Get (4);
  //      A     C
  //      |     |    flow 1: B -> A
  //      B  E  D    flow 2: D -> C     (flow 1 & flow 2 are independent / without interference)

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(1.0, 2.0, 0.0));
  positionAlloc->Add(Vector(1.0, 1.0, 0.0));
  positionAlloc->Add(Vector(3.6, 2.0, 0.0));
  positionAlloc->Add(Vector(3.6, 1.0, 0.0));
  positionAlloc->Add(Vector(2.5, 1.0, 0.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Install wireless devices
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  if(prop_loss == 1){
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(1.6));
  }
  else if(prop_loss == 2){
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel","Frequency",DoubleValue(5.15e9),"SystemLoss",DoubleValue(1),"MinLoss",DoubleValue(0));
  }

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel.Create ());
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC

  WifiHelper wifiA;
  wifiA.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifiA.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (nA_txRate),
                                "ControlMode",StringValue (nA_txRate));
  NetDeviceContainer nADevice = wifiA.Install (wifiPhy, wifiMac, nA);

  WifiHelper wifiB;
  wifiB.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifiB.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (nB_txRate),
                                "ControlMode",StringValue (nB_txRate));
  NetDeviceContainer nBDevice = wifiB.Install (wifiPhy, wifiMac, nB);

  WifiHelper wifiC;
  wifiC.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifiC.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (nC_txRate),
                                "ControlMode",StringValue (nC_txRate));
  NetDeviceContainer nCDevice = wifiC.Install (wifiPhy, wifiMac, nC);

  WifiHelper wifiD;
  wifiD.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifiD.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (nD_txRate),
                                "ControlMode",StringValue (nD_txRate));
  NetDeviceContainer nDDevice = wifiD.Install (wifiPhy, wifiMac, nD);

  WifiHelper wifiE;
  wifiE.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifiE.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (nE_txRate),
                                "ControlMode",StringValue (nE_txRate));
  NetDeviceContainer nEDevice = wifiE.Install (wifiPhy, wifiMac, nE);
  // Install TCP/IP stack & assign IP addresses
  AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (aodv, 10);
  InternetStackHelper internet;
  internet.SetRoutingHelper(aodv);
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer nAInterface;
  nAInterface = ipv4.Assign (nADevice);
  Ipv4InterfaceContainer nBInterface;
  nBInterface = ipv4.Assign (nBDevice);
  Ipv4InterfaceContainer nCInterface;
  nCInterface = ipv4.Assign (nCDevice);
  Ipv4InterfaceContainer nDInterface;
  nDInterface = ipv4.Assign (nDDevice);

  // Install applications: two CBR stream
  // flow1: B -> A; flow2: D -> C
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink1 = Socket::CreateSocket (nodes.Get(0), tid);
  InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny(), 80);
  recvSink1->Bind (local1);
  recvSink1->SetRecvCallback(MakeCallback(&ReceivePacket));

  Ptr<Socket> source1 = Socket::CreateSocket (nodes.Get(1), tid);
  InetSocketAddress remote1 = InetSocketAddress (nAInterface.GetAddress(0, 0), 80);
  source1->Connect (remote1);
  Simulator::ScheduleWithContext (source1->GetNode()->GetId(),
                                       Seconds (1.001), &GenerateTraffic,
                                       source1, packetSize, sim_time-1.001, interval);

  //TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink2 = Socket::CreateSocket (nodes.Get(2), tid);
  InetSocketAddress local2 = InetSocketAddress (Ipv4Address::GetAny(), 80);
  recvSink2->Bind (local2);
  recvSink2->SetRecvCallback(MakeCallback(&ReceivePacket));

  Ptr<Socket> source2 = Socket::CreateSocket (nodes.Get(3), tid);
  InetSocketAddress remote2 = InetSocketAddress (nCInterface.GetAddress(0, 0), 80);
  source2->Connect (remote2);
  Simulator::ScheduleWithContext (source2->GetNode()->GetId(),
                                      Seconds (1.0015), &GenerateTraffic,
                                      source2, packetSize, sim_time-1.0015, interval);

  // The workaround for adding echo apps before the real data stream begins
  uint16_t  echoPort = 9;
  UdpEchoClientHelper echoClientHelper (Ipv4Address ("10.0.0.1"), echoPort);
  echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
  ApplicationContainer pingApps;

  // again using different start times to workaround Bug 388 and Bug 912
  for (uint16_t i = 1; i < 4; i ++){
    echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001+0.005*(i-1))));
    pingApps.Add (echoClientHelper.Install (nodes.Get (i)));
  }

  //Config::ConnectWithoutContext("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx" , MakeCallback(&MACTx1));
  //Config::ConnectWithoutContext("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx" , MakeCallback(&MACTx2));
  //Config::ConnectWithoutContext("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx" , MakeCallback(&MACRx1));
  //Config::ConnectWithoutContext("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx" , MakeCallback(&MACRx2));

  Config::ConnectWithoutContext("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx1));
  Config::ConnectWithoutContext("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx2));
  Config::ConnectWithoutContext("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx1));
  Config::ConnectWithoutContext("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx2));

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  //wifiPhy.EnablePcap("wifiA",nADevice);
  wifiPhy.EnablePcap("wifiB",nBDevice);
  //wifiPhy.EnablePcap("wifiC",nCDevice);
  //wifiPhy.EnablePcap("wifiD",nDDevice);
  //wifiPhy.EnablePcap("wifiE", nEDevice);
  // Run simulation for sim_time seconds
  double run_time = sim_time + 1;
  Simulator::Stop (Seconds (run_time));
  Simulator::Run ();

  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      if(outputmode == 1){
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Tx Packets:   " << i->second.txPackets << "\n";
        std::cout << "  Rx Packets:   " << i->second.rxPackets << "\n";
        std::cout << "  Lost Packets:   " << i->second.lostPackets << "\n";
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (sim_time - 1) / 1000 / 1000  << " Mbps\n";
        std::cout << "  Packet Rate: " << i->second.rxPackets * 1.0 / (sim_time - 1) << " Pkt/Sec" << "\n";

        if(i->first == 6){
          real_pr2 = i->second.rxPackets * 1.0 / (sim_time - 1);
        }
      }
      else if(outputmode == 2){
        if(i->first == 11){
          std::cout << sim_time << " " << i->second.rxPackets * 1.0 / (sim_time - 1) << " " << N_pkt_2_ob/(sim_time - 1) * (1 + (on/N_on)/(ia/N_ia)) << "\n";
        }
      }
    }
  // Cleanup
  Simulator::Destroy ();

  if(outputmode == 1) {
    est_pr2 = N_pkt_2_ob/(sim_time - 1) *(1 + (on/N_on)/(ia/N_ia));

    std::cout << Npkt_1 << " " << Npkt_1_rx << " " << N_pkt_1_ob << " " << on/N_on << ia/N_ia << std::endl;
    std::cout << Npkt_2 << " " << Npkt_2_rx << " " << N_pkt_2_ob << std::endl;
    std::cout << N_pkt_2_ob/(sim_time - 1) * (1 + (on/N_on)/(ia/N_ia)) << std::endl;
    //std::cout << pow((real_pr2 - est_pr2),2) << std::endl;
  }

  return 0;
}
