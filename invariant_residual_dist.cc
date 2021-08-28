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


#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>
using namespace ns3;

//define all the global variables here:
double Npkt[3], Npkt_ob[3], Npkt_rx[3], Npkt_drop[3], Npkt_ob_r[3];
bool prev_pkt[3] = {true,true,true};
bool status[3] = {false,false,false};
bool p1[3] = {false,false,false}; bool p2[3] = {false,false,false}; // the two-step memory for CG estimation
bool cg_true[9];
bool cg[9];
double cg_count[9];

int NiC2[3] = {0,0,0}; // the observable values of N_{i->C2} for r_i's estimation
int N2[9] = {0,0,0,0,0,0,0,0,0}; // for debug: record all the cases of N_{1->12, 1->13, 2->12, 2->23, 3->13, 3->23}

int N2r[9] = {0,0,0,0,0,0,0,0,0};
int N3[3] = {0,0,0};
double r[3] = {0,0,0};
std::map<int,int> srctable;

//int flow_src[3] = {8,7,0}; int flow_dst[3] = {12,11,1};    // three links indep
//int flow_src[3] = {12,4,9}; int flow_dst[3] = {8,8,10};    // the most HN one
int flow_src[3] = {8,6,4}; int flow_dst[3] = {12,10,5};
//int flow_src[3] = {0,2,7}; int flow_dst[3] = {1,3,11};

double prev_on[3] = {0,0,0}; double prev_ia[3] = {0,0,0};
double ia[3] = {0,0,0}; double on[3] = {0,0,0};
double N_on[3] = {0,0,0}; double N_ia[3] = {0,0,0};
std::ofstream ia_time1;
std::ofstream ia_time2;
std::ofstream ia_time3;

std::ofstream on_time1;
std::ofstream on_time2;
std::ofstream on_time3;

std::ofstream res;
std::ofstream res_trans;

//helper functions:
double Get_r_denom(bool* arr, int i) {
  double denom = 0;
  for(int j=i*3; j<=i*3+2; j++){
    if(arr[j] == 0) {
      denom += Npkt_ob[j-i*3];
    }
  }
  return denom;
}

double GetSoP(bool* arr, double* r, int i) {
  double sop = 0.0;
  bool v[3] = {0,0,0};
  //std::cout << "the link: " << i << std::endl;
  for (int j= i*3; j<= i*3+2; j++){
    if (arr[j] == 0){
      sop += r[j-i*3];
      //std::cout << j-i*3 << std::endl;
      v[j-i*3] = 1; // indicating all the links can co-Tx with link i
    }
  }
  for (int k=0; k<=2; k++){
    if (v[k] == 1) {
      for (int l= k; l<=2; l++){
        if(arr[k*3+l] == 0 && v[l] == 1){
          sop += r[k]*r[l];
          //std::cout << k << l << std::endl;
        }
      }
    }
  }
  return sop;
}

int SrctoLink(int src) { // maps the src id to the link id for unicasting network
  int* idx = std::find(std::begin(flow_src),std::end(flow_src),src);
  return std::distance(flow_src,idx);
}

void AddLink(std::map<int, int> &srctable, int uid, int src){
  std::map<int,int>::iterator it;
  it = srctable.find(uid);
  if(it == srctable.end()) {//if not seen before
    //std::cout << "SrctoLink: " << SrctoLink(src);
    srctable.insert(std::pair<int, int>(uid,SrctoLink(src)));
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

// helper function for counting number of idle links when a specific link starts Tx:
int NumActive (bool *arr) {
  int Num = 0;
  for(int i=0; i<=2; i++){
    if(arr[i] == true)
      {Num ++;}
  }
  return Num;
}
// helper function for finding the only one link that is active
int getOneActive (bool *arr) {
  int index = 0;
  for(int i=0; i<=2; i++){
    if(arr[i] == true)
      index = i;
  }
  return index;
}
// helper function for finding the two links that are co-active
int getLeftActive(bool *arr, int f) {
  static int idx[2] = {0,0}; int iter = 0; int return_value = 0;
  for(int i=0; i<=2; i++){
    if(arr[i] == true) {
      idx[iter] = i;
      iter ++;
    }
  }
  for(int i=0; i<=1; i++) {
    if(idx[i] != f) {
      return_value = idx[i];
    }
  }
  return return_value;
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

int ContextToLinkId(std::string context, bool x) {  // x=0 for Tx; x=1 for Rx
  std::string sub = context.substr (10);
  int pos = sub.find ("/Device");
  int nodeid = atoi(sub.substr (0, pos).c_str ());
  int* idx; int return_value;
  if(x==0){ // node is src, then one-to-one mapping to linkid is doable:
    idx = std::find(std::begin(flow_src), std::end(flow_src), nodeid);
    if(idx != std::end(flow_src))
      return_value = std::distance(flow_src,idx);
  }
  else if(x==1){ // node is dst, then cannot directly map to linkid
    return_value = nodeid;
  }
  return return_value;
}

// Tracer callbacks:
void PhyTx(std::string context, Ptr<const Packet> p, double txPowerW) {
  if (p->GetSize() == 1064) {
    //NS_LOG_UNCOND ("PhyTx link " << ContextToLinkId(context,0) << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3]<< std::endl);
    //NS_LOG_UNCOND("PhyTx src node: " << ContextToLinkId(context,1));

    // increment the packet number tracker:
    Npkt[ContextToLinkId(context,0)] ++;

    if(NumActive(status) == 0){
      Npkt_ob[ContextToLinkId(context,0)] ++;
    }
    else if(NumActive(status) == 1) { // N_{j -> i,j}, increment the NiC2[i]
      NiC2[getOneActive(status)] ++;
      N2[getOneActive(status)*3+ContextToLinkId(context,0)] ++;
    }
    else if(NumActive(status) == 2) { // N_{i,j -> i,j,k}, check how the product-term of r's looks like with hidden nodes
      N3[ContextToLinkId(context,0)] ++;
    }

    // update the Tx status vector while estimating the CG:
    std::copy(p1,p1+3,p2);  //p2 = p1
    std::copy(status,status+3,p1); //p1 = status
    status[ContextToLinkId(context,0)] = true;

    if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
      int i = getOneActive(status), j = getOneActive(p2);
      cg_count[3*i+j]++; cg_count[3*j+i]++;
    }

    if(prev_ia[ContextToLinkId(context,0)] != 0){
      // record the past ia time for the link that just starts a TX:
      if(ContextToLinkId(context,0) == 0) {
        ia_time1 << (Simulator::Now().GetSeconds() - prev_ia[ContextToLinkId(context,0)]) << " ";
      }
      else if(ContextToLinkId(context,0) == 1) {
        ia_time2 << (Simulator::Now().GetSeconds() - prev_ia[ContextToLinkId(context,0)]) << " ";
      }
      else if(ContextToLinkId(context,0) == 2) {
        ia_time3 << (Simulator::Now().GetSeconds() - prev_ia[ContextToLinkId(context,0)]) << " ";
      }
      ia[ContextToLinkId(context,0)] += (Simulator::Now().GetSeconds() - prev_ia[ContextToLinkId(context,0)]);
      N_ia[ContextToLinkId(context,0)] ++;
      // update the time-stamp for the start of an on time:
      prev_on[ContextToLinkId(context,0)] = Simulator::Now().GetSeconds();
    }

    if(ContextToLinkId(context,0) == 2){
      if(status[1] == 0){
        float residual_trans = Simulator::Now().GetSeconds() - prev_ia[1];
        res_trans << residual_trans << " ";
      }
    }

  }
}

// tracer function to keep track of collided packet's finish time
void PhyTxEnd(int linkID){
  //NS_LOG_UNCOND("PHY-TX_END time=" << Simulator::Now().GetSeconds() << " link=" << linkID);
  //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << std::endl);
  //std::cout << "PhyTxEnd: " << linkID << std::endl;
  if(NumActive(status) == 1) {
    Npkt_ob_r[linkID] ++; // increment N_{i->phi} as link i finish its Tx (with UNsuccess Reception)
  }
  else if(NumActive(status) == 2) {
    N2r[getLeftActive(status,linkID)*3+linkID] ++;
  }

  std::copy(p1,p1+3,p2);  //p2 = p1
  std::copy(status,status+3,p1); //p1 = status
  status[linkID] = false;
  prev_pkt[linkID] = false; // indicating next event that previous Tx is failed on link idx

  if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
    int i = getOneActive(status), j = getOneActive(p2);
    cg_count[3*i+j]++; cg_count[3*j+i]++;
  }
  /*
  if(linkID == 0) {
    on_time1 << (Simulator::Now().GetSeconds() - prev_on[linkID]) << " ";
  }
  else if(linkID == 1) {
    on_time2 << (Simulator::Now().GetSeconds() - prev_on[linkID]) << " ";
  }
  else if(linkID == 2) {
    on_time3 << (Simulator::Now().GetSeconds() - prev_on[linkID]) << " ";
  }
  */
  //record the past on times of link that just finish its Tx:
  on[linkID] += (Simulator::Now().GetSeconds() - prev_on[linkID]);
  N_on[linkID] += 1;
  // update the time-stamp for the start of current pkt Tx
  prev_ia[linkID] = Simulator::Now().GetSeconds();
}

void PhyRx(std::string context, Ptr<const Packet> p) {
  if (p->GetSize() == 1064) {
    if(FindLink(srctable,p->GetUid())!= -1) {
      int idx = FindLink(srctable,p->GetUid());
      if( ContextToLinkId(context,1) == flow_dst[idx] ) {
        //NS_LOG_UNCOND ("PhyRx link " << idx << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
        //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3]);
        //NS_LOG_UNCOND("PhyRx dst node: " << ContextToLinkId(context,1) << std::endl);

        if(NumActive(status) == 1) {
          Npkt_ob_r[idx] ++; // increment N_{i->phi} as link i finish its Tx (with success Reception)
        }
        else if(NumActive(status) == 2) {
          N2r[getLeftActive(status,idx)*3+idx] ++;
        }
        // update the Tx status vector while estimating the CG:
        std::copy(p1,p1+3,p2);  //p2 = p1
        std::copy(status,status+3,p1); //p1 = status
        status[idx] = false;
        prev_pkt[idx] = true;

        if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
          int i = getOneActive(status), j = getOneActive(p2);
          cg_count[3*i+j]++; cg_count[3*j+i]++;
        }


        //increment the packet number tracker:
        Npkt_rx[idx] ++;
        Delsrc(srctable, p->GetUid());

        //record the past on times of link that just finish its Tx:
        if(idx == 0) {
          on_time1 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
        }
        else if(idx == 1) {
          on_time2 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
        }
        else if(idx == 2) {
          on_time3 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
        }

        on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]);
        N_on[idx] += 1;
        // update the time-stamp for the start of current pkt Tx
        prev_ia[idx] = Simulator::Now().GetSeconds();
      }
    }
  }
}

void PhyRxDrop(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  if(p->GetSize() == 1064) {
    if(FindLink(srctable,p->GetUid())!= -1) {
      int idx = FindLink(srctable,p->GetUid());
      if( ContextToLinkId(context,1) == flow_dst[idx] ) {
        //NS_LOG_UNCOND("PHY-RX-Drop time=" << Simulator::Now().GetSeconds() << " node=" << idx << " for " << p->GetUid() << " size=" << p->GetSize() << " reason: " << reason);
        //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << std::endl);
        //std::cout << "PhyRxDrop: " << FindLink(srctable,p->GetUid()) << std::endl;

        Npkt_drop[idx] ++;
        if (reason == 3) {
          if(NumActive(status) == 1) {
            Npkt_ob_r[idx] ++; // increment N_{i->phi} as link i finish its Tx (with success Reception)
          }
          else if(NumActive(status) == 2) {
            N2r[getLeftActive(status,idx)*3+idx] ++;
          }

          // update the Tx status vector while estimating the CG:
          std::copy(p1,p1+3,p2);  //p2 = p1
          std::copy(status,status+3,p1); //p1 = status
          status[idx] = false;
          prev_pkt[idx] = false; // indicating next event that previous Tx is failed on link idx
          if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
            int i = getOneActive(status), j = getOneActive(p2);
            cg_count[3*i+j]++; cg_count[3*j+i]++;
          }

          if(idx == 0) {
            on_time1 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
          }
          else if(idx == 1) {
            on_time2 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
          }
          else if(idx == 2) {
            on_time3 << (Simulator::Now().GetSeconds() - prev_on[idx]) << " ";
          }
          //record the past on times of link that just finish its Tx:
          on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]);
          N_on[idx] += 1;
          // update the time-stamp for the start of current pkt Tx
          prev_ia[idx] = Simulator::Now ().GetSeconds ();

        }
        else if(reason == 2 || reason == 5) {
          Simulator::Schedule(Seconds(0.0014), &PhyTxEnd, idx);
        }
      }
    }
  }
}

void BackoffTrace(std::string context, uint32_t newVal){
  if(ContextToLinkId(context,0) == 1) {}
  //NS_LOG_UNCOND("Backoff time=" << Simulator::Now().GetSeconds() << "node=" << ContextToLinkId(context,0) << "val=" <<newVal);
}

void CWTrace(std::string context, uint32_t oldVal, uint32_t newVal){
  if(ContextToLinkId(context,0) == 1) {}
  //NS_LOG_UNCOND("CW time=" << Simulator::Now().GetSeconds() << "node=" << ContextToLinkId(context,0) << "newval=" <<newVal << "oldval=" << oldVal);
}

void GetResidualOFF(float t, int linkID){
  float residual = 0;
  if(status[linkID] == 0){ // if the link of interest is counting down
    residual = Simulator::Now().GetSeconds() - prev_ia[linkID];
  }
  std::cout << t << " " << residual << "\n";
  if(residual != 0){
    res << residual << " ";
  }
}

//the main function:
int main(int argc, char *argv[]){

  double sim_time = 20;
  uint16_t prop_loss = 1;
  uint16_t outputmode = 1;
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 100000; // a sufficient large enough number
  double mean = 0.02;
	double bound = 0.0;

  std::fill_n(cg, 9, 1); // initialize the contention graph as a fully-connected one.
  std::fill_n(cg_count, 9, 0); // initialize the counter for each co-Tx pairs
  std::fill_n(Npkt, 3, 0); std::fill_n(Npkt_ob_r, 3, 0); std::fill_n(Npkt_rx, 3, 0); std::fill_n(Npkt_ob, 3, 0); std::fill_n(Npkt_drop, 3, 0);

  CommandLine cmd;
  cmd.AddValue ("sim_time", "simulation time in seconds", sim_time);
  cmd.AddValue ("prop_loss", "select the propagation loss model: 1 for RangePropagationLossModel; 2 for Friis model", prop_loss);
  cmd.AddValue ("outputmode", "selecte the output format: 1 reader friendly 2 for matlab", outputmode);
  cmd.AddValue ("ia_mean", "the mean value of the exponentially distributed pkt inter-arrival times", mean);
  cmd.Parse(argc, argv);

  Ptr<ExponentialRandomVariable> interval = CreateObject<ExponentialRandomVariable> ();
	interval->SetAttribute ("Mean", DoubleValue (mean));
	interval->SetAttribute ("Bound", DoubleValue (bound));

  NodeContainer nodes;
  nodes.Create(16);

  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                "MinX",DoubleValue(0.0),
                                "MinY",DoubleValue(0.0),
                                "DeltaX",DoubleValue(1.0),
                                "DeltaY",DoubleValue(1.0),
                                "GridWidth",UintegerValue(4),
                                "LayoutType",StringValue("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Install wireless devices
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  if(prop_loss == 1){
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(1.1));
  }
  else if(prop_loss == 2){
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel","Frequency",DoubleValue(5.15e9),"SystemLoss",DoubleValue(1),"MinLoss",DoubleValue(0));
    //channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
  }

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

  // Install TCP/IP stack & assign IP addresses
  /*
  AodvHelper aodv;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (aodv, 10);
  */
  InternetStackHelper internet;
  //internet.SetRoutingHelper(aodv);
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer nodeInterface;
  nodeInterface = ipv4.Assign (devices);

  //Install Applications: 4 single-hop flows
  //int flow_src[4] = {0,4,6,8}; int flow_dst[4] = {1,5,3,5};
  for(int i=0; i<=2; i++) {
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
  for (uint16_t i = 0; i <= 2; i ++){
    UdpEchoClientHelper echoClientHelper (nodeInterface.GetAddress(flow_dst[i],0), echoPort);
    echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClientHelper.SetAttribute ("Interval", TimeValue(Seconds (0.1)));
    echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
    echoClientHelper.SetAttribute ("StartTime", TimeValue(Seconds (0.001+0.005*(i))));
    pingApps.Add (echoClientHelper.Install (nodes.Get (flow_src[i])));
  }

  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/MaxCw",UintegerValue(511));
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/MinCw",UintegerValue(511));


  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",MakeCallback(&PhyRxDrop));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/BackoffTrace",MakeCallback(&BackoffTrace));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/CwTrace",MakeCallback(&CWTrace));

  float LO = 1.1; float HI = float(sim_time);
  for(int i = 0; i<5000; i++) {
    float r = LO + static_cast<float>(rand())/(static_cast <float> (RAND_MAX/(HI-LO)));
    //std::cout << r << "\n";
    Simulator::Schedule(Seconds(r),&GetResidualOFF, r, 1);
  }

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  //wifiPhy.EnablePcap("wifiA",devices.Get(0));
  wifiPhy.EnablePcap("wifi_link1",devices.Get(12));
  wifiPhy.EnablePcap("wifi_link2",devices.Get(4));
  wifiPhy.EnablePcap("wifi_link3",devices.Get(9));

  ia_time1.open("ia_dist1.txt");
  ia_time2.open("ia_dist2.txt");
  ia_time3.open("ia_dist3.txt");
  on_time1.open("on_dist1.txt");
  on_time2.open("on_dist2.txt");
  on_time3.open("on_dist3.txt");

  res.open("res.txt");
  res_trans.open("res_trans.txt");


  Simulator::Stop(Seconds (sim_time));
  Simulator::Run();

  ia_time1 << -1 << "\n";
  ia_time2 << -1 << "\n";
  ia_time3 << -1 << "\n";
  on_time1 << -1 << "\n";
  on_time2 << -1 << "\n";
  on_time3 << -1 << "\n";
  res << -1 << "\n";
  res_trans << -1 << "\n";

  ia_time1.close();
  ia_time2.close();
  ia_time3.close();
  on_time1.close();
  on_time2.close();
  on_time3.close();
  res.close();
  res_trans.close();

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
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (sim_time - 1) / 1000 / 1000  << " Mbps\n";
        std::cout << "  Packet Rate: " << i->second.rxPackets * 1.0 / (sim_time - 1) << " Pkt/Sec" << "\n";
      }

    }
  // Cleanup
  Simulator::Destroy ();

  // contention graph estimation & link packet rate estimation
  std::fill_n(cg, 9, 1); // initialize the contention graph as a fully-connected one.
  int N_total_pkt = 0;
  for(int i =0; i<=8; i++){
    N_total_pkt += cg_count[i];
  }

  for(int i=0; i<=8; i++){
    //cg_count[i] = cg_count[i]/N_total_pkt;
    if(cg_count[i] >= 0.01 && i % 3 != i / 3)
      {cg[i] = 0;}
  }

  // computation of r and r_empirical:
  for (int i=0; i<=2; i++){
    r[i] = static_cast<double>(NiC2[i])/Get_r_denom(cg,i);
  }

  double collision[3] = {0,0,0}; double total_pkt[3] = {0,0,0}; double sr = 0;
  for (int i=0; i<=2; i++){
    collision[i] += Npkt_drop[i];
    total_pkt[i] += Npkt[i];
    sr += Npkt_ob[i];
  }

  if(outputmode == 1) {
    std::cout << "Npkt: " << Npkt[0] << " " << Npkt[1]<< " " << Npkt[2] << std::endl;
    std::cout << "Npkt_ob: " << Npkt_ob[0] << " " << Npkt_ob[1] << " " << Npkt_ob[2] << std::endl;
    std::cout << "True pkt rates: " << Npkt[0]/(sim_time-1) << " "
              << Npkt[1]/(sim_time-1) << " " << Npkt[2]/(sim_time-1) << "\n";

    std::cout << "Computed pkt rates: " << Npkt_ob[0]*(1+GetSoP(cg,r,0))/(sim_time-1) << " "
              << Npkt_ob[1]*(1+GetSoP(cg,r,1))/(sim_time-1) << " " << Npkt_ob[2]*(1+GetSoP(cg,r,2))/(sim_time-1) << "\n";
  }

  return 0;
}
