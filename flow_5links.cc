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
#include <map>
#include <algorithm>
using namespace ns3;

//define all the global variables
//int Nflow = 5;
double Npkt[5] = {0,0,0,0,0}; double Npkt_ob[5] = {0,0,0,0,0}; double Npkt_rx[5] = {0,0,0,0,0}; double Npkt_drop[5] = {0,0,0,0,0};

double prev_on[5] = {0,0,0,0,0}; double prev_ia[5] = {0,0,0,0,0};
double ia[5] = {0,0,0,0,0}; double on[5] = {0,0,0,0,0};
double N_on[5] = {0,0,0,0,0}; double N_ia[5] = {0,0,0,0,0};

bool status[5] = {false,false,false,false,false};
bool p1[5] = {false,false,false,false,false}; bool p2[5] = {false,false,false,false,false}; // the two-step memory for CG estimation
bool cg[25];
double cg_count[25];

int NiC2[5] = {0,0,0,0,0}; // the observable values of N_{i->C2} for r_i's estimation
double r[5] = {0,0,0,0,0};
double r_em[5] = {0,0,0,0,0};
std::map<int,int> srctable;

double Get_r_denom(bool* arr, int i) {
  double denom = 0;
  for(int j=i*5; j<=i*5+4; j++){
    if(arr[j] == 0) {
      denom += Npkt_ob[j-i*5];
    }
  }
  return denom;
}

double GetSoP(bool* arr, double* r, int i) {
  double sop = 0.0;
  bool v[5] = {0,0,0,0,0};
  std::cout << "the link: " << i << std::endl;
  for (int j= i*5; j<= i*5+4; j++){
    if (arr[j] == 0){
      sop += r[j-i*5];
      std::cout << j-i*5 << std::endl;
      v[j-i*5] = 1; // indicating all the links can co-Tx with link i
    }
  }
  for (int k=0; k<=4; k++){
    if (v[k] == 1) {
      for (int l= k; l<=4; l++){
        if(arr[k*5+l] == 0 && v[l] == 1){
          sop += r[k]*r[l];
          std::cout << k << l << std::endl;
        }
      }
    }
  }
  return sop;
}


void AddSrc(std::map<int, int> &srctable, int uid, int src){
  std::map<int,int>::iterator it;
  it = srctable.find(uid);
  if(it == srctable.end()) //if not seen before
    srctable.insert(std::pair<int, int>(uid,src));
}

// helper function for finding the source node for each pkt
int FindSrc(std::map<int, int> &srctable, int uid){
  std::map<int,int>::iterator it;
  it = srctable.find(uid);
  if(it != srctable.end())
    return it->second;
  else
    return -1;  //no entry for the current uid
}

// helper function for updating the srctable by erasing the used pkt mapping
void Delsrc(std::map<int, int> &srctable, int uid){
  std::map<int,int>::iterator it;
  it = srctable.find(uid);
  if(it != srctable.end())
    srctable.erase(it);
}

// helper function for counting number of idle links when a specific link starts Tx:
int NumActive (bool *arr) {
  int Num = 0;
  for(int i=0; i<=4; i++){
    if(arr[i] == true)
      {Num ++;}
  }
  return Num;
}
// helper function for finding the only one link that is active
int getOneActive (bool *arr) {
  int index = 0;
  for(int i=0; i<=4; i++){
    if(arr[i] == true)
      index = i;
  }
  return index;
}

// helper functions for packet generation/reception:
void ReceivePacket (Ptr<Socket> socket)
{
	Address from;
	Ptr<Packet> p;
  while ((p = socket->RecvFrom(from)))
    {
			if (p->GetSize() == 1000) {
      	//NS_LOG_UNCOND("Received a Pkt!");
			}
    }
}
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Ptr<ExponentialRandomVariable> pktInterval)
{
  if (pktCount > 0)
    {
			Ptr<Packet> data_packet = Create<Packet> (pktSize);
      AddSrc(srctable,data_packet->GetUid(),socket->GetNode()->GetId());
			socket->Send(data_packet);
      Simulator::Schedule (Seconds(0.0014+pktInterval->GetValue()), &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close();
    }
}

uint32_t ContextToNodeId(std::string context) {
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

// Tracer callbacks:
void PhyTx(std::string context, Ptr<const Packet> p, double txPowerW) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyTx node " << ContextToNodeId(context) << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

    //record the past ia times of link that about to Tx:
    if(prev_ia[ContextToNodeId(context)] != 0){
      ia[ContextToNodeId(context)] += (Simulator::Now().GetSeconds() - prev_ia[ContextToNodeId(context)]);
      N_ia[ContextToNodeId(context)] += 1;

    }
    // update the time-stamp for the start of current pkt Tx
    prev_on[ContextToNodeId(context)] = Simulator::Now ().GetSeconds ();

    // increment the packet number tracker:
    Npkt[ContextToNodeId(context)] ++;

    if(NumActive(status) == 0){
      Npkt_ob[ContextToNodeId(context)] ++;
    }
    else if(NumActive(status) == 1) { // N_{j -> i,j}, increment the NiC2[i]
      NiC2[getOneActive(status)] ++;
    }

    // update the Tx status vector while estimating the CG:
    std::copy(p1,p1+5,p2);  //p2 = p1
    std::copy(status,status+5,p1); //p1 = status
    status[ContextToNodeId(context)] = true;

    if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
      int i = getOneActive(status), j = getOneActive(p2);
      cg_count[5*i+j]++; cg_count[5*j+i]++;
    }

  }
}

// tracer function to keep track of collided packet's finish time
void PhyTxEnd(int nodeID){
  //NS_LOG_UNCOND("PHY-TX_END time=" << Simulator::Now().GetSeconds() << " node=" << nodeID);
  //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);
  std::copy(p1,p1+5,p2);  //p2 = p1
  std::copy(status,status+5,p1); //p1 = status
  status[nodeID] = false;
  if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
    int i = getOneActive(status), j = getOneActive(p2);
    cg_count[5*i+j]++; cg_count[5*j+i]++;
  }

  //record the past on times of link that just finish its Tx:
  on[nodeID] += (Simulator::Now().GetSeconds() - prev_on[nodeID]);
  N_on[nodeID] += 1;
  // update the time-stamp for the start of current pkt Tx
  prev_ia[nodeID] = Simulator::Now ().GetSeconds ();

}

void PhyRx(std::string context, Ptr<const Packet> p) {
  if (p->GetSize() == 1064) {
    if(FindSrc(srctable,p->GetUid())!= -1 && ContextToNodeId(context)-5 == FindSrc(srctable,p->GetUid())) {
      //NS_LOG_UNCOND ("PhyRx node " << ContextToNodeId(context)-5 << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
      //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

      // update the Tx status vector while estimating the CG:
      std::copy(p1,p1+5,p2);  //p2 = p1
      std::copy(status,status+5,p1); //p1 = status
      status[ContextToNodeId(context)-5] = false;

      if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
        int i = getOneActive(status), j = getOneActive(p2);
        cg_count[5*i+j]++; cg_count[5*j+i]++;
      }

      //record the past on times of link that just finish its Tx:
      on[ContextToNodeId(context)-5] += (Simulator::Now().GetSeconds() - prev_on[ContextToNodeId(context)-5]);
      N_on[ContextToNodeId(context)-5] += 1;
      // update the time-stamp for the start of inter-arrival interval:
      prev_ia[ContextToNodeId(context)-5] = Simulator::Now ().GetSeconds ();

      //increment the packet number tracker:
      Npkt_rx[ContextToNodeId(context)-5] ++;
      Delsrc(srctable, p->GetUid());
    }
  }
}

void PhyRxDrop(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  if(p->GetSize() == 1064) {
    if(FindSrc(srctable,p->GetUid())!= -1 && ContextToNodeId(context)-5 == FindSrc(srctable,p->GetUid())) {
      //NS_LOG_UNCOND("PHY-RX-Drop time=" << Simulator::Now().GetSeconds() << " node=" << ContextToNodeId (context)-5 << " for " << p->GetUid() << " size=" << p->GetSize() << " reason: " << reason);
      //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);
      Npkt_drop[ContextToNodeId(context)-5] ++;
      if (reason == 3) {
        // update the Tx status vector while estimating the CG:
        std::copy(p1,p1+5,p2);  //p2 = p1
        std::copy(status,status+5,p1); //p1 = status
        status[ContextToNodeId(context)-5] = false;
        if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
          int i = getOneActive(status), j = getOneActive(p2);
          cg_count[5*i+j]++; cg_count[5*j+i]++;
        }

        //record the past on times of link that just finish its Tx:
        on[ContextToNodeId(context)-5] += (Simulator::Now().GetSeconds() - prev_on[ContextToNodeId(context)-5]);
        N_on[ContextToNodeId(context)-5] += 1;
        // update the time-stamp for the start of current pkt Tx
        prev_ia[ContextToNodeId(context)-5] = Simulator::Now ().GetSeconds ();

      }
      else if(reason == 2 || reason == 5) {
        Simulator::Schedule(Seconds(0.0014), &PhyTxEnd, ContextToNodeId(context)-5);
      }
    }
  }
}

int main (int argc, char *argv[]){

  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10000; // a sufficient large enough number
  double mean = 0.02;
	double bound = 0.0;

  std::fill_n(cg, 25, 1); // initialize the contention graph as a fully-connected one.
  std::fill_n(cg_count, 25, 0); // initialize the counter for each co-Tx pairs

  double sim_time = 20;
  uint16_t prop_loss = 1;
  uint16_t outputmode = 1;
  double delta = 0.0;

  CommandLine cmd;
  cmd.AddValue ("sim_time", "simulation time in seconds", sim_time);
  cmd.AddValue ("delta", "The scale that determines the distance between nodes", delta);
  cmd.AddValue ("prop_loss", "select the propagation loss model: 1 for RangePropagationLossModel; 2 for Friis model", prop_loss);
  cmd.AddValue ("outputmode", "selecte the output format: 1 reader friendly 2 for matlab", outputmode);
  cmd.AddValue ("ia_mean", "the mean value of the exponentially distributed pkt inter-arrival times", mean);
  cmd.Parse(argc, argv);

  Ptr<ExponentialRandomVariable> interval = CreateObject<ExponentialRandomVariable> ();
	interval->SetAttribute ("Mean", DoubleValue (mean));
	interval->SetAttribute ("Bound", DoubleValue (bound));

  NodeContainer nodes;
  nodes.Create (10);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(1.0, 4.0, 0.0));
  positionAlloc->Add(Vector(1.0, 3.0, 0.0));
  positionAlloc->Add(Vector(1.0, 1.5-delta, 0.0));
  positionAlloc->Add(Vector(6.0, 4.0, 0.0));
  positionAlloc->Add(Vector(6.0, 3.0, 0.0));

  positionAlloc->Add(Vector(1.0, 3.5, 0.0));
  positionAlloc->Add(Vector(1.0, 2.5, 0.0));
  positionAlloc->Add(Vector(1.0, 2.0-delta, 0.0));
  //positionAlloc->Add(Vector(1.0, 1.0, 0.0));
  positionAlloc->Add(Vector(6.0, 3.5, 0.0));
  positionAlloc->Add(Vector(6.0, 2.5, 0.0));

  mobility.SetPositionAllocator(positionAlloc);
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
    //channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  }

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel.Create ());
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue ("OfdmRate6Mbps"),
                                "ControlMode",StringValue ("OfdmRate6Mbps"));

  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

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
  Ipv4InterfaceContainer nodeInterface;
  nodeInterface = ipv4.Assign (devices);


  //Install Applications: 5 single-hop flows
  int flow_src[5] = {0,1,2,3,4}; int flow_dst[5] = {5,6,7,8,9};
  //flow 1: Node 1 -> Node 2  (Node 1 -> Node 4)
  //flow 2: Node 2 -> Node 4  (Node 2 -> Node 5)
  //flow 3: Node 3 -> Node 5  (Node 3 -> Node 6)
  for(int i=0; i<=4; i++) {
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get(flow_dst[i]), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> source = Socket::CreateSocket(nodes.Get(flow_src[i]), tid);
    InetSocketAddress remote = InetSocketAddress(nodeInterface.GetAddress(flow_dst[i], 0), 80);
    source->Connect (remote);
    Simulator::ScheduleWithContext (source->GetNode()->GetId(),
                                         Seconds (1.001 + i*(0.0005)), &GenerateTraffic,
                                         source, packetSize, numPackets, interval);
  }

  // we also use separate UDP applications that will send a single
  // packet before the CBR flows start.
  // This is a workround for the lack of perfect ARP, see Bug 187
  // http://www.nsnam.org/bugzilla/show_bug.cgi?id=187
  uint16_t  echoPort = 9;
  ApplicationContainer pingApps;

  // again using different start times to workaround Bug 388 and Bug 912
  for (uint16_t i = 0; i <= 4; i ++){
    UdpEchoClientHelper echoClientHelper (nodeInterface.GetAddress(flow_dst[i],0), echoPort);
    echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
    echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
    echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001+0.005*(i))));
    pingApps.Add (echoClientHelper.Install (nodes.Get (flow_src[i])));
  }

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",MakeCallback(&PhyRxDrop));

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  wifiPhy.EnablePcap("wifiA",devices.Get(0));
  wifiPhy.EnablePcap("wifiB",devices.Get(1));
  wifiPhy.EnablePcap("wifiA_recv",devices.Get(5));

  Simulator::Stop (Seconds (sim_time));
  Simulator::Run ();

  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      if(outputmode == 1) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Tx Packets:   " << i->second.txPackets << "\n";
        std::cout << "  Rx Packets:   " << i->second.rxPackets << "\n";
        std::cout << "  Lost Packets:   " << i->second.lostPackets << "\n";
        //std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (sim_time - 1) / 1000 / 1000  << " Mbps\n";
        std::cout << "  Packet Rate: " << i->second.rxPackets * 1.0 / (sim_time - 1) << " Pkt/Sec" << "\n";
      }
    }
  // Cleanup
  Simulator::Destroy ();
  // contention graph estimation & link packet rate estimation
  std::fill_n(cg, 25, 1); // initialize the contention graph as a fully-connected one.
  int N_total_pkt = 0;
  for(int i =0; i<=24; i++){
    N_total_pkt += cg_count[i];
  }

  for(int i=0; i<=24; i++){
    cg_count[i] = cg_count[i]/N_total_pkt;
    if(cg_count[i] >= 0.01)
      {cg[i] = 0;}
  }

  // computation of r and r_empirical:
  for (int i=0; i<=4; i++){
    r[i] = static_cast<double>(NiC2[i])/Get_r_denom(cg,i);
    //r_em[i] = (on[i]/N_on[i])/(ia[i]/N_ia[i]);

  }

  double collision[5] = {0,0,0,0,0}; double total_pkt[5] = {0,0,0,0,0}; double sr = 0;
  for (int i=0; i<=4; i++){
    collision[i] += Npkt_drop[i];
    total_pkt[i] += Npkt[i];
    sr += Npkt_ob[i];
  }

  if(outputmode == 1) {
    std::cout << "Npkt: " << Npkt[0] << " " << Npkt[1] << " " << Npkt[2] << " " << Npkt[3] << " " << Npkt[4] << std::endl;
    std::cout << "Npkt_rx: " << Npkt_rx[0] << " " << Npkt_rx[1] << " " << Npkt_rx[2] << " " << Npkt_rx[3] << " " << Npkt_rx[4] << std::endl;
    std::cout << "Npkt_drop: " << Npkt_drop[0] << " " << Npkt_drop[1] << " " << Npkt_drop[2] << " " << Npkt_drop[3] << " " << Npkt_drop[4] << std::endl;
    std::cout << "Npkt_ob: " << Npkt_ob[0] << " " << Npkt_ob[1] << " " << Npkt_ob[2] << " " << Npkt_ob[3] << " " << Npkt_ob[4] << std::endl;
    //std::cout << "NiC2: " << NiC2[0] << " " << NiC2[1] << " " << NiC2[2] << " " << NiC2[3] << " " << NiC2[4] << std::endl;
    std::cout << "r_est: " << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " " << r[4] << std::endl;
    //std::cout << "r_empirical:" << r_em[0] << " " << r_em[1] << " " << r_em[2] << " " << r_em[3] << " " << r_em[4] << std::endl;

    std::cout << "Computed pkt volume: " << Npkt_ob[0]*(1+GetSoP(cg,r,0)) << " " << Npkt_ob[1]*(1+GetSoP(cg,r,1)) << " " << Npkt_ob[2]*(1+GetSoP(cg,r,2)) << " "
              << Npkt_ob[3]*(1+GetSoP(cg,r,3)) << " " << Npkt_ob[4]*(1+GetSoP(cg,r,4)) << "\n";
  }
  else if(outputmode == 2){
    double Npkt_link1 = Npkt_ob[0]*(1 + GetSoP(cg,r,0)); double Npkt_link3 = Npkt_ob[2]*(1 + GetSoP(cg,r,2)); double Npkt_link5 = Npkt_ob[4]*(1 + GetSoP(cg,r,4));
    double Npkt_link2 = Npkt_ob[1]*(1 + GetSoP(cg,r,1)); double Npkt_link4 = Npkt_ob[3]*(1 + GetSoP(cg,r,3));
    //std::cout << sim_time << " " << collision/total_pkt << " " << Npkt[0]*1.0/(sim_time - 1) << " "
    //          << Npkt[1]*1.0/(sim_time - 1) << " " << Npkt[2]*1.0/(sim_time - 1) << " "<< Npkt[3]*1.0/(sim_time - 1) << " " << Npkt[0]*1.0/(sim_time - 1) << " "
    //          << Npkt_link1*1.0/(sim_time - 1) << " " << Npkt_link2*1.0/(sim_time - 1) << " " << Npkt_link3*1.0/(sim_time - 1) << " " << Npkt_link4*1.0/(sim_time - 1) <<
    //          << " " << Npkt_link5*1.0/(sim_time - 1) << "\n";

    std::cout << sim_time << " " << mean << " "
              << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " " << r[4] << " "
              << total_pkt[0]*1.0 << " " << total_pkt[1]*1.0 << " " << total_pkt[2]*1.0 << " " << total_pkt[3]*1.0 << " " << total_pkt[4]*1.0 << " "
              << collision[0]*1.0 << " " << collision[1]*1.0 << " " << collision[2]*1.0 << " " << collision[3]*1.0 << " " << collision[4]*1.0 << " "
              << sr << " "
              << Npkt[0]*1.0/(sim_time - 1) << " " << Npkt[1]*1.0/(sim_time - 1) << " " << Npkt[2]*1.0/(sim_time - 1) << " "
              << Npkt[3]*1.0/(sim_time - 1) << " " << Npkt[4]*1.0/(sim_time - 1) << " "
              << Npkt_link1*1.0/(sim_time - 1) << " " << Npkt_link2*1.0/(sim_time - 1) << " " << Npkt_link3*1.0/(sim_time - 1) << " "
              << Npkt_link4*1.0/(sim_time - 1) << " " << Npkt_link5*1.0/(sim_time - 1) << "\n";
  }
  for(int i=0; i<=24; i++){ std::cout << cg[i] << ' ';}
  return 0;
}
