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
#define PI 3.14159265

//define all the global variables
//int Nflow = 5;
double Npkt[5] = {0,0,0,0,0}; double Npkt_ob[5] = {0,0,0,0,0}; double Npkt_rx[5] = {0,0,0,0,0}; double Npkt_drop[5] = {0,0,0,0,0};

double ET_on[5] = {0,0,0,0,0}; bool intact_H[5] = {false,false,false,false,false};

double N_enqueue[5] = {0,0,0,0,0}; double N_backlog[5] = {0,0,0,0,0};

double prev_on[5] = {0,0,0,0,0}; double prev_ia[5] = {0,0,0,0,0};
double ia[5] = {0,0,0,0,0}; double on[5] = {0,0,0,0,0};
double N_on[5] = {0,0,0,0,0}; double N_ia[5] = {0,0,0,0,0};

bool status[5] = {false,false,false,false,false};
bool p1[5] = {false,false,false,false,false}; bool p2[5] = {false,false,false,false,false}; // the two-step memory for CG estimation
bool cg[25];
double cg_count[25];

int NiC2[5] = {0,0,0,0,0}; // the observable values of N_{i->C2} for r_i's estimation
double r[5] = {0,0,0,0,0};
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

/*
double GetSoP(bool* arr, double* r, int i) {
  double sop = 0.0;
  bool v[5] = {0,0,0,0,0};
  //std::cout << "the link: " << i << std::endl;
  for (int j= i*5; j<= i*5+4; j++){
    if (arr[j] == 0){
      sop += r[j-i*5];
      //std::cout << j-i*5 << std::endl;
      v[j-i*5] = 1; // indicating all the links can co-Tx with link i
    }
  }
  for (int k=0; k<=4; k++){
    if (v[k] == 1) {
      for (int l= k; l<=4; l++){
        if(arr[k*5+l] == 0 && v[l] == 1){
          sop += r[k]*r[l];
          //std::cout << k << l << std::endl;
        }
      }
    }
  }
  return sop;
}
*/

void get_rest_cg(int i, bool* cg_i, bool* rest_cg){

  std::fill_n(rest_cg, 5, 0);
  for(int j=i+1; j<=4; j++) {
    rest_cg[j] = cg_i[j];
  }
}

bool* add_cur_set(bool* cg_cur, int j){
  bool* cg_cur_new = new bool[5];
  std::copy(cg_cur,cg_cur+5,cg_cur_new);
  cg_cur_new[j] = 1;
  return cg_cur_new;
}

double SoP_helper(int j, double* r, bool* cg_i_rest, bool* cg_cur) {

  if(j == 4){
    return 1.0;
  }

  else{
    double sum = 1.0;
    for(int k = 0; k<=4; k++){
      if(cg_i_rest[k] != 0){ // if the rest_cg is empty, then we directly skip here
        bool flag = 1;
        //if the new-added link k is indep with all existing links:
        for(int l=0; l<=4; l++){
          if(cg_cur[l]!=0){ // for all the links in the set:
            //if(j==2 && k==4){NS_LOG_UNCOND(j << k << l);}
            if(cg[5*l+k] == 1){ // if link k interferes with any one of them:
              flag = 0;
            }
          }
        }
        if(flag == 1){
          bool* cg_rest_new = new bool[5];
          get_rest_cg(k,cg_i_rest,cg_rest_new);
          //NS_LOG_UNCOND(k << " + " << cg_cur[0] << " " << cg_cur[1] << " " << cg_cur[2]);
          //NS_LOG_UNCOND(r[k]*SoP_helper(k,r,cg_rest_new,add_cur_set(cg_cur,k)));
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
  double sop = 0.0; bool cg_i[5];
  std::fill_n(cg_i, 5, 0);
  for (int j= i*5; j<= i*5+4; j++){
    if (arr[j] == 0){
      cg_i[j-i*5] = 1; // indicating all the links can co-Tx with link i
    }
  }

  for(int j=0; j<=4; j++){
    if(cg_i[j] != 0){
      bool* cg_cur = new bool[5];
      std::fill_n(cg_cur,5,0);

      bool* cg_i_rest = new bool[5];
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
			if (p->GetSize() >= 1000) {
      	//NS_LOG_UNCOND("Received a Pkt!");
			}
    }
}
//uint32_t pktSize,
static void GenerateTraffic (Ptr<Socket> socket, Ptr<UniformRandomVariable> pktSize,
                             uint32_t pktCount, Ptr<ExponentialRandomVariable> pktInterval, double mean, double mean2)
{
  if (pktCount > 0)
    {
      //RngSeedManager::SetRun(2);
      if(socket->GetNode()->GetId() == 0) {
        pktInterval->SetAttribute ("Mean", DoubleValue (mean));
      }
      else if(socket->GetNode()->GetId() == 1) {
        pktInterval->SetAttribute ("Mean", DoubleValue(mean2));
      }
      else {
        pktInterval->SetAttribute ("Mean", DoubleValue(mean));
      }
      pktSize->SetAttribute("Min",DoubleValue(1000.0));
      pktSize->SetAttribute("Max",DoubleValue(1200.0));

      uint32_t N = (uint32_t)(pktSize->GetValue());
			Ptr<Packet> data_packet = Create<Packet> (N);
      AddSrc(srctable,data_packet->GetUid(),socket->GetNode()->GetId());
			socket->Send(data_packet);
      Simulator::Schedule (Seconds(pktInterval->GetValue()), &GenerateTraffic,
                           socket, pktSize, pktCount - 1, pktInterval, mean, mean2);
    }
  else
    {
      socket->Close();
    }
}

int ContextToNodeId(std::string context) {
  std::string sub = context.substr (10);
  int pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

// Tracer callbacks:
void PhyTx(std::string context, Ptr<const Packet> p, double txPowerW) {
  if (p->GetSize() >= 1064) {
    //NS_LOG_UNCOND ("PhyTx node " << ContextToNodeId(context) << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
    //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

    //By default, the MAC header is corrupted:
    intact_H[ContextToNodeId(context)] = false;
    //record the past ia times of link that about to Tx:
    if(prev_ia[ContextToNodeId(context)] != 0){
      ia[ContextToNodeId(context)] += (Simulator::Now().GetSeconds() - prev_ia[ContextToNodeId(context)]);
      N_ia[ContextToNodeId(context)] += 1;
    }
    // update the time-stamp for the start of current pkt Tx
    prev_on[ContextToNodeId(context)] = Simulator::Now().GetSeconds ();

    // increment the packet number tracker:
    Npkt[ContextToNodeId(context)] ++;
    if(N_enqueue[ContextToNodeId(context)] > 1) {N_backlog[ContextToNodeId(context)] ++;}

    if(NumActive(status) == 0){
      Npkt_ob[ContextToNodeId(context)] ++;
      intact_H[ContextToNodeId(context)] = true; // The case where MAC header is intact
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
      //if((i==0 && j==1) || (j==0 && i==1)) {NS_LOG_UNCOND("Flow 1&2 cg_count ++ !!!");}
    }

  }
}

// tracer function to keep track of collided packet's finish time
void PhyTxEnd(int nodeID){
  //NS_LOG_UNCOND("PHY-TX_END time=" << Simulator::Now().GetSeconds() << " node=" << nodeID);
  //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

  //record the past on times of link that just finish its Tx:
  if(prev_on[nodeID] != 0){
    on[nodeID] += (Simulator::Now().GetSeconds() - prev_on[nodeID]);
    N_on[nodeID] += 1;
    if(intact_H[nodeID] == true){
      ET_on[nodeID] += (Simulator::Now().GetSeconds() - prev_on[nodeID]); // Collect the T_on for all pkt with intact MAC header
    }
  }
  // update the time-stamp for the start of current pkt Tx
  prev_ia[nodeID] = Simulator::Now ().GetSeconds ();

  std::copy(p1,p1+5,p2);  //p2 = p1
  std::copy(status,status+5,p1); //p1 = status
  status[nodeID] = false;
  //NS_LOG_UNCOND("The p2: " << p2[0] << " " << p2[1] << " " << p2[2] << " " << p2[3] << " " << p2[4] << std::endl);
  //NS_LOG_UNCOND("The p1: " << p1[0] << " " << p1[1] << " " << p1[2] << " " << p1[3] << " " << p1[4] << std::endl);
  //NS_LOG_UNCOND("The status: " << status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);
  if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
    int i = getOneActive(status), j = getOneActive(p2);
    cg_count[5*i+j]++; cg_count[5*j+i]++;
    //if((i==0 && j==1) || (j==0 && i==1)) {NS_LOG_UNCOND("Flow 1&2 cg_count ++ !!!");}
  }
}

void PhyRx(std::string context, Ptr<const Packet> p) {
  if (p->GetSize() >= 1064) {
    int idx = FindSrc(srctable,p->GetUid());
    if(FindSrc(srctable,p->GetUid())!= -1 && ContextToNodeId(context)-5 == FindSrc(srctable,p->GetUid())) {
      //NS_LOG_UNCOND ("PhyRx node " << ContextToNodeId(context)-5 << " at " << Simulator::Now ().GetSeconds () << " for " << p->GetUid() << " Length " << p->GetSize());
      //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

      //record the past on times of link that just finish its Tx:
      if(prev_on[idx] != 0){
        on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]);
        N_on[idx] += 1;
        if(intact_H[idx] == true){
          ET_on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]); // Collect the T_on for all pkt with intact MAC header
        }
      }
      // update the time-stamp for the start of current pkt Tx
      prev_ia[idx] = Simulator::Now().GetSeconds();

      // update the Tx status vector while estimating the CG:
      std::copy(p1,p1+5,p2);  //p2 = p1
      std::copy(status,status+5,p1); //p1 = status
      status[ContextToNodeId(context)-5] = false;

      if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
        int i = getOneActive(status), j = getOneActive(p2);
        cg_count[5*i+j]++; cg_count[5*j+i]++;
        //if((i==0 && j==1) || (j==0 && i==1)) {NS_LOG_UNCOND("Flow 1&2 cg_count ++ !!!");}
      }

      //increment the packet number tracker:
      Npkt_rx[ContextToNodeId(context)-5] ++;
      Delsrc(srctable, p->GetUid());
    }
  }
}

void PhyRxDrop(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  if(p->GetSize() >= 1064) {
    int idx = FindSrc(srctable,p->GetUid());
    if(FindSrc(srctable,p->GetUid())!= -1 && ContextToNodeId(context)-5 == FindSrc(srctable,p->GetUid())) {

        //NS_LOG_UNCOND("PHY-RX-Drop time=" << Simulator::Now().GetSeconds() << " node=" << ContextToNodeId (context)-5 << " for " << p->GetUid() << " size=" << p->GetSize() << " reason: " << reason);
        //NS_LOG_UNCOND(status[0] << " " << status[1] << " " << status[2] << " " << status[3] << " " << status[4] << std::endl);

      Npkt_drop[ContextToNodeId(context)-5] ++;
      if (reason == 3) {

        //record the past on times of link that just finish its Tx:
        if(prev_on[idx] != 0){
          on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]);
          N_on[idx] += 1;
          if(intact_H[idx] == true){
            ET_on[idx] += (Simulator::Now().GetSeconds() - prev_on[idx]); // Collect the T_on for all pkt with intact MAC header
          }
        }
        // update the time-stamp for the start of current pkt Tx
        prev_ia[idx] = Simulator::Now ().GetSeconds ();

        // update the Tx status vector while estimating the CG:
        std::copy(p1,p1+5,p2);  //p2 = p1
        std::copy(status,status+5,p1); //p1 = status
        status[ContextToNodeId(context)-5] = false;
        if(NumActive(p2) == 1 && NumActive(status) == 1 && NumActive(p1) > 1){
          int i = getOneActive(status), j = getOneActive(p2);
          cg_count[5*i+j]++; cg_count[5*j+i]++;
          //if((i==0 && j==1) || (j==0 && i==1)) {NS_LOG_UNCOND("Flow 1&2 cg_count ++ !!!");}
        }
      }
      else if(reason == 2 || reason == 5) {
        double p_duration = (p->GetSize()+19)*8.0/(6*1000*1000);
        Simulator::Schedule(Seconds(p_duration), &PhyTxEnd, ContextToNodeId(context)-5);
      }
    }
  }
}

void CheckEnqueue(std::string context, Ptr<const WifiMacQueueItem> item) {
  //NS_LOG_UNCOND("Enueue=" << Simulator::Now().GetSeconds() << " Tx node ID=" << ContextToNodeId(context) << " Q_size: " << N_enqueue[ContextToNodeId(context)]);
  N_enqueue[ContextToNodeId(context)] ++;
}

void CheckDequeue(std::string context, Ptr<const WifiMacQueueItem> item) {
  //NS_LOG_UNCOND("Dequeue=" << Simulator::Now().GetSeconds() << " Tx node ID=" << ContextToNodeId(context) << " Q_size: " << N_enqueue[ContextToNodeId(context)]);
  N_enqueue[ContextToNodeId(context)] --;
}

void CW_trace(std::string context, uint32_t CW_now, uint32_t CW_next) {
  //NS_LOG_UNCOND("CWTrace=" << Simulator::Now().GetSeconds() << " Tx node ID=" << ContextToNodeId(context) << " CW_now: " << CW_now << " CW_next: " << CW_next);
}

int main (int argc, char *argv[]){

  //uint32_t packetSize = 1000; // bytes
  Ptr<UniformRandomVariable> packetSize = CreateObject<UniformRandomVariable> ();
  packetSize->SetAttribute("Min",DoubleValue(1000.0));
  packetSize->SetAttribute("Max",DoubleValue(1200.0));
  uint32_t numPackets = 100000; // a sufficient large enough number
  double mean = 0.02;
  double mean2 = 0.02;
	double bound = 0.0;

  std::fill_n(cg, 25, 1); // initialize the contention graph as a fully-connected one.
  std::fill_n(cg_count, 25, 0); // initialize the counter for each co-Tx pairs

  double sim_time = 20;
  double delta2 = 3.3;
  double delta4 = 3.3;
  uint16_t prop_loss = 1;
  uint16_t outputmode = 1;
  uint16_t Topology_Run = 1;
  uint16_t Sim_Run = 1;
  uint16_t top = 1;

  CommandLine cmd;
  cmd.AddValue ("sim_time", "simulation time in seconds", sim_time);
  cmd.AddValue ("prop_loss", "select the propagation loss model: 1 for RangePropagationLossModel; 2 for Friis model", prop_loss);
  cmd.AddValue ("delta2", "Tx position of link 2", delta2);
  cmd.AddValue ("delta4", "Tx position of link 4", delta4);
  cmd.AddValue ("outputmode", "select the output format: 1 reader friendly 2 for matlab", outputmode);
  cmd.AddValue ("ia_mean", "the mean value of the exponentially distributed pkt inter-arrival times", mean);
  cmd.AddValue ("ia_mean2", "the mean value of the exponentially distributed pkt inter-arrival times for node 2", mean2);
  cmd.AddValue ("TopologyRun", "the mean value of the exponentially distributed pkt inter-arrival times", Topology_Run);
  cmd.AddValue ("top", "the topology index for test with HN collision", top);
  cmd.Parse(argc, argv);

  Ptr<ExponentialRandomVariable> interval = CreateObject<ExponentialRandomVariable> ();
	interval->SetAttribute ("Mean", DoubleValue (mean));
	interval->SetAttribute ("Bound", DoubleValue (bound));

  NodeContainer txer,rxer;
  txer.Create (5); rxer.Create(5);

  MobilityHelper mobility_tx,mobility_rx;
  Ptr<ListPositionAllocator> positionAlloc_tx = CreateObject<ListPositionAllocator>();
  Ptr<ListPositionAllocator> positionAlloc_rx = CreateObject<ListPositionAllocator>();

  if(top == 0) {  //The case without HN yet direct interference (for sim_T.sh testing)
    positionAlloc_tx->Add(Vector(1.7, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(2.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(2.3, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.4, 2.0, 0.0));

    positionAlloc_rx->Add(Vector(1.7, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(2.0, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(2.3, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(7.4, 2.4, 0.0));
  }

  if(top == 1) {
    positionAlloc_tx->Add(Vector(0.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.3, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(5.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(9.0, 2.0, 0.0));

    positionAlloc_rx->Add(Vector(0.0, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(1.3, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(5.0, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 2.4, 0.0));
    positionAlloc_rx->Add(Vector(9.0, 2.4, 0.0));
  }
  else if(top == 2) {
    positionAlloc_tx->Add(Vector(1.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.0, 4.05, 0.0));
    positionAlloc_tx->Add(Vector(3.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(9.0, 2.0, 0.0));

    positionAlloc_rx->Add(Vector(1.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(1.0, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(3.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(9.0, 3.0, 0.0));
  }

  else if(top == 3) {
    positionAlloc_tx->Add(Vector(1.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.0, 4.05, 0.0));
    positionAlloc_tx->Add(Vector(3.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 4.05, 0.0));

    positionAlloc_rx->Add(Vector(1.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(1.0, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(3.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.05, 0.0));
  }

  else if(top == 4) {
    positionAlloc_tx->Add(Vector(1.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.0, 4.05, 0.0));
    positionAlloc_tx->Add(Vector(2.05, 3.05, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(9.0, 2.0, 0.0));

    positionAlloc_rx->Add(Vector(1.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(1.0, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(1.05, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(9.0, 3.0, 0.0));
  }

  else if(top == 5) {
    positionAlloc_tx->Add(Vector(1.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.0, 4.05, 0.0));
    positionAlloc_tx->Add(Vector(2.05, 3.05, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 4.05, 0.0));

    positionAlloc_rx->Add(Vector(1.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(1.0, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(1.05, 3.05, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.05, 0.0));
  }

  else if(top == 6) {
    positionAlloc_tx->Add(Vector(1.0, 2.0, 0.0));
    positionAlloc_tx->Add(Vector(1.0, 4.3, 0.0));
    positionAlloc_tx->Add(Vector(2.3, 3.15, 0.0));
    positionAlloc_tx->Add(Vector(-0.3, 3.15, 0.0));
    positionAlloc_tx->Add(Vector(7.0, 4.3, 0.0));

    positionAlloc_rx->Add(Vector(1.0, 3.0, 0.0));
    positionAlloc_rx->Add(Vector(1.0, 3.3, 0.0));
    positionAlloc_rx->Add(Vector(1.3, 3.15, 0.0));
    positionAlloc_rx->Add(Vector(0.7, 3.15, 0.0));
    positionAlloc_rx->Add(Vector(7.0, 3.3, 0.0));
  }

  else if(top == 7) {
    double r = 1.01; double R = 0.01;
    positionAlloc_tx->Add(Vector(1.0+r*cos(0.4*PI), 3.15+r*sin(0.4*PI), 0.0));
    positionAlloc_tx->Add(Vector(1.0+r*cos(0.8*PI), 3.15+r*sin(0.8*PI), 0.0));
    positionAlloc_tx->Add(Vector(1.0+r*cos(1.2*PI), 3.15+r*sin(1.2*PI), 0.0));
    positionAlloc_tx->Add(Vector(1.0+r*cos(1.6*PI), 3.15+r*sin(1.6*PI), 0.0));
    positionAlloc_tx->Add(Vector(1.0+r*cos(2.0*PI), 3.15+r*sin(2.0*PI), 0.0));

    positionAlloc_rx->Add(Vector(1.0+R*cos(0.4*PI), 3.15+R*sin(0.4*PI), 0.0));
    positionAlloc_rx->Add(Vector(1.0+R*cos(0.8*PI), 3.15+R*sin(0.8*PI), 0.0));
    positionAlloc_rx->Add(Vector(1.0+R*cos(1.2*PI), 3.15+R*sin(1.2*PI), 0.0));
    positionAlloc_rx->Add(Vector(1.0+R*cos(1.6*PI), 3.15+R*sin(1.6*PI), 0.0));
    positionAlloc_rx->Add(Vector(1.0+R*cos(2.0*PI), 3.15+R*sin(2.0*PI), 0.0));
  }

  mobility_tx.SetPositionAllocator(positionAlloc_tx);
  mobility_tx.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_tx.Install(txer);

  mobility_rx.SetPositionAllocator(positionAlloc_rx);
  mobility_rx.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility_rx.Install(rxer);

  /*
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
    NS_LOG_UNCOND("x: " << p.x << " y: " << p.y);

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
  */

  NodeContainer nodes(txer,rxer);

  // Install wireless devices
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  if(prop_loss == 1){
    channel.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(0.5));
  }
  else if(prop_loss == 2){
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel","Frequency",DoubleValue(5.15e9),"SystemLoss",DoubleValue(1),"MinLoss",DoubleValue(0));
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
  //AodvHelper aodv;
  //Ipv4StaticRoutingHelper staticRouting;

  //Ipv4ListRoutingHelper list;
  //list.Add (staticRouting, 0);
  //list.Add (aodv, 10);
  InternetStackHelper internet;
  //internet.SetRoutingHelper(aodv);
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer nodeInterface;
  nodeInterface = ipv4.Assign (devices);

  //Install Applications: 5 single-hop flows
  //int flow_src[5] = {0,1,2,3,4}; int flow_dst[5] = {5,6,7,8,9};
  //flow 1: Node 1 -> Node 2  (Node 1 -> Node 4)
  //flow 2: Node 2 -> Node 4  (Node 2 -> Node 5)
  //flow 3: Node 3 -> Node 5  (Node 3 -> Node 6)

  //RngSeedManager::SetRun(Sim_Run);
  for(int i=0; i<=4; i++) {
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket (rxer.Get(i), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), 80);
    recvSink->Bind (local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> source = Socket::CreateSocket(txer.Get(i), tid);
    InetSocketAddress remote = InetSocketAddress(nodeInterface.GetAddress(5+i, 0), 80);
    source->Connect (remote);
    Simulator::ScheduleWithContext (source->GetNode()->GetId(),
                                         Seconds (1.001 + i*(0.0005)), &GenerateTraffic,
                                         source, packetSize, numPackets, interval, mean, mean2);
  }

  // we also use separate UDP applications that will send a single
  // packet before the CBR flows start.
  // This is a workround for the lack of perfect ARP, see Bug 187
  // http://www.nsnam.org/bugzilla/show_bug.cgi?id=187
  uint16_t  echoPort = 9;
  ApplicationContainer pingApps;

  // again using different start times to workaround Bug 388 and Bug 912
  for (uint16_t i = 0; i <= 4; i ++){
    UdpEchoClientHelper echoClientHelper (nodeInterface.GetAddress(5+i, 0), echoPort);
    echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
    echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
    echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001+0.005*(i))));
    pingApps.Add (echoClientHelper.Install(txer.Get(i)));
  }


  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin" , MakeCallback(&PhyTx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxEnd" , MakeCallback(&PhyRx));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",MakeCallback(&PhyRxDrop));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/Queue/Enqueue", MakeCallback(&CheckEnqueue));
  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/Queue/Dequeue", MakeCallback(&CheckDequeue));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/CwTrace", MakeCallback(&CW_trace));
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/MaxCw", UintegerValue (125));
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Txop/MinCw", UintegerValue (251));
  //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/FrameCaptureModel/$ns3::SimpleFrameCaptureModel/Margin",DoubleValue(-5000));

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  wifiPhy.EnablePcap("wifiA",devices.Get(0));
  wifiPhy.EnablePcap("wifiB",devices.Get(1));

  wifiPhy.EnablePcap("wifiB_recv",devices.Get(6));
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
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (sim_time - 1) / 1000 / 1000  << " Mbps\n";
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
    int a = i % 5; int b = i / 5;
    //cg_count[i] = cg_count[i]/(Npkt_ob[a]+Npkt_ob[b]);
    if (a == b) {cg_count[i] = 0;}
  }

  /*
  double t0 = *std::min_element(cg_count,cg_count+25);
  double t1 = *std::max_element(cg_count,cg_count+25);
  //std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;
  double t0_new,t1_new;
  bool flag = 0;

  while(t0 != t0_new || t1 != t1_new) {
    if(flag == 1)
      {t0 = t0_new; t1 = t1_new;}

    t0_new = 0; t1_new = 0;
    int n0 = 0; int n1 = 0;

    for(int i=0; i<=24; i++){
      if(pow(cg_count[i]-t0,2) <= pow(cg_count[i]-t1,2)){
        t0_new += cg_count[i]; n0 ++;
      }
      else{
        t1_new += cg_count[i]; n1 ++;
      }
    }
    t0_new = t0_new / static_cast<double>(n0);
    t1_new = t1_new / static_cast<double>(n1);
    //std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;
    //std::cout << "New Threshold: " << t0_new << " and threshold: " << t1_new << std::endl;
    flag = 1;
  }

  //std::cout << "Threshold: " << t0 << " and threshold: " << t1 << std::endl;

  for(int i=0; i<=24; i++){
    if(pow(cg_count[i]-t1,2) < pow(cg_count[i]-t0,2) && (i%5 != i/5))
      {cg[i] = 0;}
  }
  */

  double count_max = *std::max_element(cg_count,cg_count+25);
  for(int i=0; i<=24; i++){
    cg_count[i] = cg_count[i]/count_max;
    if(cg_count[i] >= 0.05) // a pre-defined threshold here.
      {cg[i] = 0;}
  }

  double r_real[5] = {0,0,0,0,0};
  // computation of r and r_empirical:
  for (int i=0; i<=4; i++){
    r[i] = static_cast<double>(NiC2[i])/Get_r_denom(cg,i);
    on[i] = on[i]/N_on[i];
    ia[i] = ia[i]/N_ia[i];
    r_real[i] = on[i]/ia[i];
    //std::cout << "For link " << i << " Avg. ON: " << on[i] << " Avg. IA: " << ia[i] << std::endl;
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
    std::cout << "N_backlog:" << N_backlog[0] << " " << N_backlog[1] << " " << N_backlog[2] << " " << N_backlog[3] << " " << N_backlog[4] << " " << std::endl;
    //std::cout << "NiC2: " << NiC2[0] << " " << NiC2[1] << " " << NiC2[2] << " " << NiC2[3] << " " << NiC2[4] << std::endl;

    std::cout << "r_est: " << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " " << r[4] << std::endl;
    std::cout << "r_real: " << r_real[0] << " " << r_real[1] << " " << r_real[2] << " " << r_real[3] << " " << r_real[4] << std::endl;
    std::cout << "ON:" << on[0] << " " << on[1] << " " << on[2] << " " << on[3] << " " << on[4] << std::endl;

    std::cout << "Computed pkt volume: " << Npkt_ob[0]*(1+GetSoP(cg,r,0)) << " " << Npkt_ob[1]*(1+GetSoP(cg,r,1)) << " " << Npkt_ob[2]*(1+GetSoP(cg,r,2)) << " "
              << Npkt_ob[3]*(1+GetSoP(cg,r,3)) << " " << Npkt_ob[4]*(1+GetSoP(cg,r,4)) << "\n";

    //for(int i=0; i<= 24; i++) {std::cout << cg_count[i] << " ";} std::cout << "\n";

    std::cout << "Error:" << std::abs(Npkt_ob[0]*(1+GetSoP(cg,r,0)) - Npkt[0])/Npkt[0] << " " << std::abs(Npkt_ob[1]*(1+GetSoP(cg,r,1)) - Npkt[1])/Npkt[1] << " " << std::abs(Npkt_ob[2]*(1+GetSoP(cg,r,2)) - Npkt[2])/Npkt[2]
              << " " << std::abs(Npkt_ob[3]*(1+GetSoP(cg,r,3)) - Npkt[3])/Npkt[3] << " " << std::abs(Npkt_ob[4]*(1+GetSoP(cg,r,4)) - Npkt[4])/Npkt[4] << "\n";
    std::cout << "C_rate:" << (Npkt_drop[0]+Npkt_drop[1]+Npkt_drop[2]+Npkt_drop[3]+Npkt_drop[4])/(Npkt[0]+Npkt[1]+Npkt[2]+Npkt[3]+Npkt[4]) << std::endl;
    std::cout << "ET_on" << ET_on[0]/Npkt_ob[0] << " " << ET_on[1]/Npkt_ob[1] << " " << ET_on[2]/Npkt_ob[2] << " " << ET_on[3]/Npkt_ob[3] << " " << ET_on[4]/Npkt_ob[4] << std::endl;



  }
  else if(outputmode == 2){
    double Npkt_link1 = Npkt_ob[0]*(1 + GetSoP(cg,r,0)); double Npkt_link3 = Npkt_ob[2]*(1 + GetSoP(cg,r,2)); double Npkt_link5 = Npkt_ob[4]*(1 + GetSoP(cg,r,4));
    double Npkt_link2 = Npkt_ob[1]*(1 + GetSoP(cg,r,1)); double Npkt_link4 = Npkt_ob[3]*(1 + GetSoP(cg,r,3));
    //std::cout << sim_time << " " << collision/total_pkt << " " << Npkt[0]*1.0/(sim_time - 1) << " "
    //          << Npkt[1]*1.0/(sim_time - 1) << " " << Npkt[2]*1.0/(sim_time - 1) << " "<< Npkt[3]*1.0/(sim_time - 1) << " " << Npkt[0]*1.0/(sim_time - 1) << " "
    //          << Npkt_link1*1.0/(sim_time - 1) << " " << Npkt_link2*1.0/(sim_time - 1) << " " << Npkt_link3*1.0/(sim_time - 1) << " " << Npkt_link4*1.0/(sim_time - 1) <<
    //          << " " << Npkt_link5*1.0/(sim_time - 1) << "\n";

    std::cout << sim_time << " " << delta2 << " " << mean << " "
              << Npkt[0]*1.0/(sim_time - 1) << " " << Npkt[1]*1.0/(sim_time - 1) << " " << Npkt[2]*1.0/(sim_time - 1) << " "
              << Npkt[3]*1.0/(sim_time - 1) << " " << Npkt[4]*1.0/(sim_time - 1) << " "
              << Npkt_ob[0]*1.0/(sim_time - 1) << " " << Npkt_ob[1]*1.0/(sim_time - 1) << " " << Npkt_ob[2]*1.0/(sim_time - 1) << " "
              << Npkt_ob[3]*1.0/(sim_time - 1) << " " << Npkt_ob[4]*1.0/(sim_time - 1) << " "
              << collision[0]*1.0/(sim_time - 1) << " " << collision[1]*1.0/(sim_time - 1) << " " << collision[2]*1.0/(sim_time - 1) << " "
              << collision[3]*1.0/(sim_time - 1) << " " << collision[4]*1.0/(sim_time - 1) << " "
              << Npkt_link1*1.0/(sim_time - 1) << " " << Npkt_link2*1.0/(sim_time - 1) << " " << Npkt_link3*1.0/(sim_time - 1) << " "
              << Npkt_link4*1.0/(sim_time - 1) << " " << Npkt_link5*1.0/(sim_time - 1) << " "
              << std::abs(Npkt_ob[0]*(1+GetSoP(cg,r,0)) - Npkt[0])/Npkt[0] << " " << std::abs(Npkt_ob[1]*(1+GetSoP(cg,r,1)) - Npkt[1])/Npkt[1] << " " << std::abs(Npkt_ob[2]*(1+GetSoP(cg,r,2)) - Npkt[2])/Npkt[2]
              << " " << std::abs(Npkt_ob[3]*(1+GetSoP(cg,r,3)) - Npkt[3])/Npkt[3] << " " << std::abs(Npkt_ob[4]*(1+GetSoP(cg,r,4)) - Npkt[4])/Npkt[4] << " "
              << (Npkt_drop[0]+Npkt_drop[1]+Npkt_drop[2]+Npkt_drop[3]+Npkt_drop[4])/(Npkt[0]+Npkt[1]+Npkt[2]+Npkt[3]+Npkt[4]) << " "
              << ET_on[0]/Npkt_ob[0] << " " << ET_on[1]/Npkt_ob[1] << " " << ET_on[2]/Npkt_ob[2] << " " << ET_on[3]/Npkt_ob[3] << " " << ET_on[4]/Npkt_ob[4] << "\n";
  }
  //for(int i=0; i<=24; i++){ std::cout << cg_count[i] << ' ';}
  return 0;
}
