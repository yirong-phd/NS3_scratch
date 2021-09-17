#include "ns3/log.h"
#include "ns3/config.h"
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
#include "ns3/netanim-module.h"

#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
using namespace ns3;

#define PI 3.14159265
std::map<int,int> srctable;
double Npkt[10], Npkt_ob[10], NiC2[10], Npkt_rx[10], Npkt_drop[10];
bool status[10];

double r[10];

bool p1[10], p2[10]; // the two-step memory for CG estimation
bool cg_true[100];
bool cg[100];
double cg_count[100];

double Get_r_denom(bool* arr, int i) {
  double denom = 0;
  for(int j=i*10; j<=i*10+9; j++){
    if(arr[j] == 0) {
      denom += Npkt_ob[j-i*10];
    }
  }
  return denom;
}

void get_rest_cg(int i, bool* cg_i, bool* rest_cg){

  std::fill_n(rest_cg, 10, 0);
  for(int j=i+1; j<=9; j++) {
    rest_cg[j] = cg_i[j];
  }
}

bool* add_cur_set(bool* cg_cur, int j){
  bool* cg_cur_new = new bool[10];
  std::copy(cg_cur,cg_cur+10,cg_cur_new);
  cg_cur_new[j] = 1;
  return cg_cur_new;
}

double SoP_helper(int j, double* r, bool* cg_i_rest, bool* cg_cur) {

  if(j == 9){
    //std::cout << std::endl;
    return 1.0;
  }

  else{
    double sum = 1.0;
    for(int k = 0; k<=9; k++){
      if(cg_i_rest[k] != 0){ // if the rest_cg is empty, then we directly skip here
        bool flag = 1;
        //if the new-added link k is indep with all existing links:
        for(int l=0; l<=9; l++){
          if(cg_cur[l]!=0){ // for all the links in the set:
            if(cg[10*l+k] == 1){ // if link k interferes with any one of them:
              flag = 0;
            }
          }
        }
        if(flag == 1){
          bool* cg_rest_new = new bool[10];
          get_rest_cg(k,cg_i_rest,cg_rest_new);
          //std::cout << k << " ";
          sum += r[k]*SoP_helper(k,r,cg_rest_new,add_cur_set(cg_cur,k));
          delete[] cg_rest_new;
        }
      }
    }
    delete[] cg_cur;
    return sum;
  }
}


double GetSoP(bool* arr, double* r, int i) {
  double sop = 0.0; bool cg_i[10];
  std::fill_n(cg_i, 10, 0);
  for (int j= i*10; j<= i*10+9; j++){
    if (arr[j] == 0){
      cg_i[j-i*10] = 1; // indicating all the links can co-Tx with link i
    }
  }

  for(int j=0; j<=9; j++){
    if(cg_i[j] != 0){
      bool* cg_cur = new bool[10];
      std::fill_n(cg_cur,10,0);

      bool* cg_i_rest = new bool[10];
      get_rest_cg(j, cg_i, cg_i_rest);

      double new_sop = r[j]*(SoP_helper(j, r, cg_i_rest, add_cur_set(cg_cur,j)));
      //NS_LOG_UNCOND("Link: " << i << " has sop for " << j << " as: " << new_sop);
      sop += new_sop;
      delete[] cg_cur;
      delete[] cg_i_rest;
    }
  }
  return sop;
}

void AddLink(std::map<int, int> &srctable, int uid, int src){
  std::map<int,int>::iterator it;
  it = srctable.find(uid);
  if(it == srctable.end()) {//if not seen before
    //std::cout << "SrctoLink: " << SrctoLink(src);
    srctable.insert(std::pair<int, int>(uid,src));
  }
}

// helper function for finding the link id (mapped from source node) for each pkt
int FindLink(std::map<int, int> &srctable, int uid){
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
      //NS_LOG_UNCOND("Generate Pkt on Link: " << socket->GetNode()->GetId());

			Ptr<Packet> data_packet = Create<Packet> (pktSize);
      AddLink(srctable,data_packet->GetUid(),socket->GetNode()->GetId());
			socket->Send(data_packet);
      double interval = pktInterval->GetValue();
      Simulator::Schedule(Seconds(interval), &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close();
    }
}

// helper function for counting number of idle links when a specific link starts Tx:
int NumActive (bool *arr) {
  int Num = 0;
  for(int i=0; i<=9; i++){
    if(arr[i] == true)
      {Num ++;}
  }
  return Num;
}
// helper function for finding the only one link that is active
int getOneActive (bool *arr) {
  int index = 0;
  for(int i=0; i<=9; i++){
    if(arr[i] == true)
      index = i;
  }
  return index;
}

int ContextToLinkId(std::string context, bool x) {  // x=0 for Tx; x=1 for Rx
  std::string sub = context.substr(10);
  int pos = sub.find ("/Device");
  int nodeid = atoi(sub.substr (0, pos).c_str ());
  if(x == 0){
    return nodeid;
  }
  else{
    return (nodeid - 10);
  }
}

// Tracer callbacks:
void PhyTx(std::string context, Ptr<const Packet> p, double txPowerW) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyTx link " << ContextToLinkId(context,0) << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " <<
    //               status[4] << " " << status[5] << " " << status[6] << " " << status[7] << " " <<
    //               status[8] << " " << status[9] << std::endl);

    // increment the packet number tracker:
    Npkt[ContextToLinkId(context,0)] ++;

    if(NumActive(status) == 0){
      Npkt_ob[ContextToLinkId(context,0)] ++;
    }
    else if(NumActive(status) == 1) { // N_{j -> i,j}, increment the NiC2[i]
      NiC2[getOneActive(status)] ++;
    }

    // update the Tx status vector while estimating the CG:
    std::copy(p1,p1+10,p2);  //p2 = p1
    std::copy(status,status+10,p1); //p1 = status
    status[ContextToLinkId(context,0)] = true;

    if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) == 2){
      int i = getOneActive(status), j = getOneActive(p2);
      cg_count[10*i+j]++; cg_count[10*j+i]++;
    }
  }
}

// tracer function to keep track of collided packet's finish time
void PhyTxEnd(int linkID){
  //NS_LOG_UNCOND("PHY-TX_END time=" << Simulator::Now().GetSeconds() << " link=" << linkID);
  //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " <<
  //               status[4] << " " << status[5] << " " << status[6] << " " << status[7] << " " <<
  //               status[8] << " " << status[9] << std::endl);

  std::copy(p1,p1+10,p2);  //p2 = p1
  std::copy(status,status+10,p1); //p1 = status
  status[linkID] = false;

  if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) == 2){
    int i = getOneActive(status), j = getOneActive(p2);
    cg_count[10*i+j]++; cg_count[10*j+i]++;
  }
}

void PhyRx(std::string context, Ptr<const Packet> p) {
  if (p->GetSize() == 1064) {
    if(FindLink(srctable,p->GetUid())!= -1) {
      int idx = FindLink(srctable,p->GetUid());
      //NS_LOG_UNCOND(ContextToLinkId(context,1) << " + " << idx);
      if( ContextToLinkId(context,1) == idx ) {
        //NS_LOG_UNCOND ("PhyRx link " << idx << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
        //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " <<
        //               status[4] << " " << status[5] << " " << status[6] << " " << status[7] << " " <<
        //               status[8] << " " << status[9] << std::endl);

        // update the Tx status vector while estimating the CG:
        std::copy(p1,p1+10,p2);  //p2 = p1
        std::copy(status,status+10,p1); //p1 = status
        status[idx] = false;

        if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) == 2){
          int i = getOneActive(status), j = getOneActive(p2);
          cg_count[10*i+j]++; cg_count[10*j+i]++;
        }

        //increment the packet number tracker:
        Npkt_rx[idx] ++;
        Delsrc(srctable, p->GetUid());
      }
    }
  }
}

void PhyRxDrop(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  if(p->GetSize() == 1064) {
    if(FindLink(srctable,p->GetUid())!= -1) {
      int idx = FindLink(srctable,p->GetUid());
      if( ContextToLinkId(context,1) == idx ) {
        //NS_LOG_UNCOND("PHY-RX-Drop time=" << Simulator::Now().GetSeconds() << " node=" << idx << " for " << p->GetUid() << " size=" << p->GetSize() << " reason: " << reason);
        //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " <<
        //               status[4] << " " << status[5] << " " << status[6] << " " << status[7] << " " <<
        //               status[8] << " " << status[9] << std::endl);
        //std::cout << "PhyRxDrop: " << FindLink(srctable,p->GetUid()) << std::endl;
        //if(idx == 2) {NS_LOG_UNCOND("Link 2 has a packet dropped at: " << Simulator::Now().GetSeconds());}
        Npkt_drop[idx] ++;
        if (reason == 3) {
          // update the Tx status vector while estimating the CG:
          std::copy(p1,p1+10,p2);  //p2 = p1
          std::copy(status,status+10,p1); //p1 = status
          status[idx] = false;
          if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) == 2){
            int i = getOneActive(status), j = getOneActive(p2);
            cg_count[10*i+j]++; cg_count[10*j+i]++;
          }
        }
        else if(reason == 2 || reason == 5) {
          Simulator::Schedule(Seconds(0.0014), &PhyTxEnd, idx);
        }
      }
    }
  }
}

int main(int argc, char *argv[]){

  double sim_time = 20;
  uint16_t outputmode = 1;
  uint16_t Topology_Run = 1;

  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10000; // a sufficient large enough number
  double mean = 0.02;
	double bound = 0.0;

  std::fill_n(cg, 100, 1); // initialize the contention graph as a fully-connected one.
  std::fill_n(cg_count, 100, 0); // initialize the counter for each co-Tx pairs
  std::fill_n(status, 10, 0);
  std::fill_n(p1, 10, 0);
  std::fill_n(p2, 10, 0);

  std::fill_n(r,10,0);

  std::fill_n(Npkt, 10, 0); std::fill_n(Npkt_ob, 10, 0); std::fill_n(NiC2, 10, 0);
  std::fill_n(Npkt_drop, 10, 0); std::fill_n(Npkt_rx, 10, 0);

  CommandLine cmd;
  cmd.AddValue ("sim_time", "simulation time in seconds", sim_time);
  cmd.AddValue ("outputmode", "selecte the output format: 1 reader friendly 2 for matlab", outputmode);
  cmd.AddValue ("ia_mean", "the mean value of the exponentially distributed pkt inter-arrival times", mean);
  cmd.AddValue ("TopologyRun", "the mean value of the exponentially distributed pkt inter-arrival times", Topology_Run);
  cmd.Parse(argc, argv);

  Ptr<ExponentialRandomVariable> interval = CreateObject<ExponentialRandomVariable>();
	interval->SetAttribute ("Mean", DoubleValue (mean));
	interval->SetAttribute ("Bound", DoubleValue (bound));

  NodeContainer txer,rxer;
  txer.Create(10); // 10 links for the random graph topology of our wireless CSMA/CA network
  rxer.Create(10);

  /*
  MobilityHelper mobility_tx;
  Ptr<ListPositionAllocator> positionAlloc_tx = CreateObject<ListPositionAllocator>();
  positionAlloc_tx->Add(Vector(1.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(11.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(21.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(31.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(41.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(51.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(61.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(71.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(81.0, 1.0, 0.0));
  positionAlloc_tx->Add(Vector(91.0, 1.0, 0.0));
  mobility_tx.SetPositionAllocator(positionAlloc_tx);
  mobility_tx.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_tx.Install(txer);

  MobilityHelper mobility_rx;
  Ptr<ListPositionAllocator> positionAlloc_rx = CreateObject<ListPositionAllocator>();
  positionAlloc_rx->Add(Vector(1.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(11.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(21.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(31.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(41.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(51.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(61.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(71.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(81.0, 1.5, 0.0));
  positionAlloc_rx->Add(Vector(91.0, 1.5, 0.0));
  mobility_rx.SetPositionAllocator(positionAlloc_rx);
  mobility_rx.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_rx.Install(rxer);
  */

  RngSeedManager::SetRun (Topology_Run);
  MobilityHelper mobility_tx,mobility_rx;
  mobility_tx.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                "X",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=5.0]"),
                                "Y",StringValue("ns3::UniformRandomVariable[Min=0.0|Max=5.0]"));

  mobility_tx.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility_tx.Install(txer);

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for(NodeContainer::Iterator it = txer.Begin(); it != txer.End(); ++it) {
    Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel> ();
    Vector p = mm->GetPosition();
    //NS_LOG_UNCOND("x: " << p.x << " y: " << p.y);

    Ptr<UniformRandomVariable> r = CreateObject<UniformRandomVariable> ();
  	r->SetAttribute ("Min", DoubleValue (0.0));
  	r->SetAttribute ("Max", DoubleValue (0.5));
    Ptr<UniformRandomVariable> theta = CreateObject<UniformRandomVariable> ();
  	theta->SetAttribute ("Min", DoubleValue (0.0));
  	theta->SetAttribute ("Max", DoubleValue (2*PI));
    double R = r->GetValue(); double Theta = theta->GetValue();
    positionAlloc->Add(Vector(p.x + R*sin(Theta), p.y + R*cos(Theta), 0.0));
  }

  mobility_rx.SetPositionAllocator(positionAlloc);
  mobility_rx.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_rx.Install(rxer);




  NodeContainer nodes(txer,rxer);

  // Install wireless devices
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  //channel.AddPropagationLoss("ns3::FriisPropagationLossModel","Frequency",DoubleValue(5.15e9),"SystemLoss",DoubleValue(1),"MinLoss",DoubleValue(0));
  channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(0.51));

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel.Create());
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue ("OfdmRate6Mbps"),
                                "ControlMode",StringValue ("OfdmRate6Mbps"));

  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);
  //NetDeviceContainer Tx_devices = wifi.Install (wifiPhy, wifiMac, txer);
  //NetDeviceContainer Rx_devices = wifi.Install (wifiPhy, wifiMac, rxer);

  InternetStackHelper Tx_internet,Rx_internet;
  Tx_internet.Install (txer);
  Rx_internet.Install (rxer);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");

  Ipv4InterfaceContainer nodeInterface;
  nodeInterface = ipv4.Assign(devices);


  // Install applications: 10 single-hop flows with random graph topology:
  for(int i=0; i<=9; i++) {
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket (rxer.Get(i), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> source = Socket::CreateSocket(txer.Get(i), tid);
    InetSocketAddress remote = InetSocketAddress(nodeInterface.GetAddress(10+i, 0), 80);
    source->Connect (remote);
    Simulator::ScheduleWithContext (source->GetNode()->GetId(),
                                         Seconds (1.001 + i*(0.0005)), &GenerateTraffic,
                                         source, packetSize, numPackets, interval);
  }

  /*
  // we also use separate UDP applications that will send a single
  // packet before the CBR flows start.
  // This is a workround for the lack of perfect ARP, see Bug 187
  // http://www.nsnam.org/bugzilla/show_bug.cgi?id=187
  uint16_t  echoPort = 9;
  ApplicationContainer pingApps;

  // again using different start times to workaround Bug 388 and Bug 912
  for (uint16_t i = 0; i <= 9; i ++){
    UdpEchoClientHelper echoClientHelper (nodeInterface.GetAddress(10+i,0), echoPort);
    echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClientHelper.SetAttribute ("Interval", TimeValue(Seconds (0.1)));
    echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
    echoClientHelper.SetAttribute ("StartTime", TimeValue(Seconds (0.001+0.005*(i))));
    pingApps.Add (echoClientHelper.Install (txer.Get(i)));
  }
  */

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",MakeCallback(&PhyRxDrop));
  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  //AnimationInterface anim("visual.xml");
  Simulator::Stop(Seconds (sim_time));
  Simulator::Run();

  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
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
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (sim_time - 1) / 1000 / 1000  << " Mbps\n";
        std::cout << "  Packet Rate: " << i->second.rxPackets * 1.0 / (sim_time - 1) << " Pkt/Sec" << "\n";
      }
  }
  // Cleanup
  Simulator::Destroy();


  // contention graph estimation & link packet rate estimation
  std::fill_n(cg, 100, 1); // initialize the contention graph as a fully-connected one.
  for(int i=0; i<=99; i++){
    int a = i % 10; int b = i / 10;
    cg_count[i] = cg_count[i]/(Npkt_ob[a]+Npkt_ob[b]);
    if (a == b) {cg_count[i] = 0;}
  }

  double t0 = *std::min_element(cg_count,cg_count+100);
  double t1 = *std::max_element(cg_count,cg_count+100);
  //std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;
  double t0_new,t1_new;
  bool flag = 0;

  while(t0 != t0_new || t1 != t1_new) {
    if(flag == 1)
      {t0 = t0_new; t1 = t1_new;}

    t0_new = 0; t1_new = 0;
    int n0 = 0; int n1 = 0;

    for(int i=0; i<=99; i++){
      if(pow(cg_count[i]-t0,2) <= pow(cg_count[i]-t1,2)){
        t0_new += cg_count[i]; n0 ++;
      }
      else{
        t1_new += cg_count[i]; n1 ++;
      }
    }
    t0_new = t0_new / static_cast<double>(n0);
    t1_new = t1_new / static_cast<double>(n1);
    std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;
    std::cout << "New Threshold: " << t0_new << " and threshold: " << t1_new << std::endl;
    flag = 1;
  }

  //std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;

  for(int i=0; i<=99; i++){
    if(pow(cg_count[i]-t1,2) < pow(cg_count[i]-t0,2) && (i%10 != i/10))
      {cg[i] = 0;}
  }


  /*
  // contention graph estimation & link packet rate estimation
  std::fill_n(cg, 100, 1); // initialize the contention graph as a fully-connected one.
  for(int i=0; i<=99; i++){
    //cg_count[i] = cg_count[i]/N_total_pkt;
    if(cg_count[i] > 50 && i % 10 != i / 10)
      {cg[i] = 0;}
  }
  */

  for (int i=0; i<=9; i++){
    r[i] = static_cast<double>(NiC2[i])/Get_r_denom(cg,i);
    //r[i] = i+1;
  }

  if(outputmode == 0) { std::cout << "Finished!" << std::endl;}
  if(outputmode == 1) {
    std::cout << "Npkt: " << Npkt[0] << " " << Npkt[1] << " " << Npkt[2] << " "
              << Npkt[3] << " " << Npkt[4] << " " << Npkt[5] << " "
              << Npkt[6] << " " << Npkt[7] << " " << Npkt[8] << " " << Npkt[9] << std::endl;
    std::cout << "Npkt_ob: " << Npkt_ob[0] << " " << Npkt_ob[1] << " " << Npkt_ob[2] << " "
              << Npkt_ob[3] << " " << Npkt_ob[4] << " " << Npkt_ob[5] << " "
              << Npkt_ob[6] << " " << Npkt_ob[7] << " " << Npkt_ob[8] << " " << Npkt_ob[9] << std::endl;
    //std::cout << "Npkt_rx: " << Npkt_rx[0] << " " << Npkt_rx[1] << " " << Npkt_rx[2] << std::endl;
    std::cout << "Npkt_drop: " << Npkt_drop[0] << " " << Npkt_drop[1] << " " << Npkt_drop[2] << " "
              << Npkt_drop[3] << " " << Npkt_drop[4] << " " << Npkt_drop[5] << " "
              << Npkt_drop[6] << " " << Npkt_drop[7] << " " << Npkt_drop[8] << " " << Npkt_drop[9] << std::endl;
    std::cout << "Computed pkt rates: " << Npkt_ob[0]*(1+GetSoP(cg,r,0)) << " "
              << Npkt_ob[1]*(1+GetSoP(cg,r,1)) << " " << Npkt_ob[2]*(1+GetSoP(cg,r,2)) << " "
              << Npkt_ob[3]*(1+GetSoP(cg,r,3)) << " " << Npkt_ob[4]*(1+GetSoP(cg,r,4)) << " "
              << Npkt_ob[5]*(1+GetSoP(cg,r,5)) << " " << Npkt_ob[6]*(1+GetSoP(cg,r,6)) << " "
              << Npkt_ob[7]*(1+GetSoP(cg,r,7)) << " " << Npkt_ob[8]*(1+GetSoP(cg,r,8)) << " "
              << Npkt_ob[9]*(1+GetSoP(cg,r,9)) << "\n";
    for(int i=0; i<=99; i++){ std::cout << cg_count[i] << ' ';}
    //for(int i=0; i<=9; i++){ std::cout << r[i] << ' ';}
  }
  else if(outputmode == 2) {
    double Npkt_link0 = Npkt_ob[0]*(1 + GetSoP(cg,r,0));
    double Npkt_link1 = Npkt_ob[1]*(1 + GetSoP(cg,r,1));
    double Npkt_link2 = Npkt_ob[2]*(1 + GetSoP(cg,r,2));
    double Npkt_link3 = Npkt_ob[3]*(1 + GetSoP(cg,r,3));
    double Npkt_link4 = Npkt_ob[4]*(1 + GetSoP(cg,r,4));
    double Npkt_link5 = Npkt_ob[5]*(1 + GetSoP(cg,r,5));
    double Npkt_link6 = Npkt_ob[6]*(1 + GetSoP(cg,r,6));
    double Npkt_link7 = Npkt_ob[7]*(1 + GetSoP(cg,r,7));
    double Npkt_link8 = Npkt_ob[8]*(1 + GetSoP(cg,r,8));
    double Npkt_link9 = Npkt_ob[9]*(1 + GetSoP(cg,r,9));

    std::cout << sim_time << " " << Topology_Run << " " << mean << " "
              << Npkt[0]*1.0/(sim_time-1) << " " << Npkt[1]*1.0/(sim_time-1) << " " << Npkt[2]*1.0/(sim_time-1) << " "
              << Npkt[3]*1.0/(sim_time-1) << " " << Npkt[4]*1.0/(sim_time-1) << " " << Npkt[5]*1.0/(sim_time-1) << " "
              << Npkt[6]*1.0/(sim_time-1) << " " << Npkt[7]*1.0/(sim_time-1) << " " << Npkt[8]*1.0/(sim_time-1) << " "
              << Npkt[9]*1.0/(sim_time-1) << " "
              << Npkt_ob[0]*1.0/(sim_time-1) << " " << Npkt_ob[1]*1.0/(sim_time-1) << " " << Npkt_ob[2]*1.0/(sim_time-1) << " "
              << Npkt_ob[3]*1.0/(sim_time-1) << " " << Npkt_ob[4]*1.0/(sim_time-1) << " " << Npkt_ob[5]*1.0/(sim_time-1) << " "
              << Npkt_ob[6]*1.0/(sim_time-1) << " " << Npkt_ob[7]*1.0/(sim_time-1) << " " << Npkt_ob[8]*1.0/(sim_time-1) << " "
              << Npkt_ob[9]*1.0/(sim_time-1) << " "
              << Npkt_drop[0]*1.0/(sim_time-1) << " " << Npkt_drop[1]*1.0/(sim_time-1) << " " << Npkt_drop[2]*1.0/(sim_time-1) << " "
              << Npkt_drop[3]*1.0/(sim_time-1) << " " << Npkt_drop[4]*1.0/(sim_time-1) << " " << Npkt_drop[5]*1.0/(sim_time-1) << " "
              << Npkt_drop[6]*1.0/(sim_time-1) << " " << Npkt_drop[7]*1.0/(sim_time-1) << " " << Npkt_drop[8]*1.0/(sim_time-1) << " "
              << Npkt_drop[9]*1.0/(sim_time-1) << " "
              << Npkt_link0*1.0/(sim_time - 1) << " " << Npkt_link1*1.0/(sim_time - 1) << " " << Npkt_link2*1.0/(sim_time - 1) << " "
              << Npkt_link3*1.0/(sim_time - 1) << " " << Npkt_link4*1.0/(sim_time - 1) << " " << Npkt_link5*1.0/(sim_time - 1) << " "
              << Npkt_link6*1.0/(sim_time - 1) << " " << Npkt_link7*1.0/(sim_time - 1) << " " << Npkt_link8*1.0/(sim_time - 1) << " "
              << Npkt_link9*1.0/(sim_time - 1)
              << std::endl;
  }
  return 0;
}
