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

int main(){
  NodeContainer txer,rxer;
  txer.Create(10); // 20 links for the random graph topology of our wireless CSMA/CA network
  rxer.Create(10);

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

  AnimationInterface anim("visual.xml");
  Simulator::Stop(Seconds (10));
  Simulator::Run();
  return 0;
}
