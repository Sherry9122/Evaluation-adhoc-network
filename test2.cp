
#include "ns3/core-module.h"
#include "ns3/propagation-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-module.h"
#include <fstream>
// #include "ns3/netanim-module.h"

using namespace ns3;

void experiment (bool enableCtsRts, int nodes, int area)
{
  UintegerValue ctsThr = (enableCtsRts ? UintegerValue (16) : UintegerValue (2200));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

  double threshold = -70;
  // double loss = (area == 100?-77:-85) - 5;
  double tx = threshold + 90;

  // 1. Create 3 nodes 
  NodeContainer adhocNodes;
  NodeContainer apNode;
  adhocNodes.Create(nodes - 1);
  apNode.Create(1);

  // 5. Install wireless devices
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", 
                                "DataMode",StringValue ("DsssRate11Mbps"), 
                                "ControlMode",StringValue ("DsssRate11Mbps"));
  
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
  wifiMac.SetType("ns3::AdhocWifiMac");

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

  wifiPhy.Set("TxPowerStart", DoubleValue(tx));
  wifiPhy.Set("TxPowerEnd", DoubleValue(tx));
  wifiPhy.Set("TxPowerLevels", UintegerValue(1));
  wifiPhy.Set("TxGain", DoubleValue(0));
  wifiPhy.Set("RxGain", DoubleValue(0));

  wifiPhy.Set("EnergyDetectionThreshold", DoubleValue(threshold));
  wifiPhy.Set("CcaMode1Threshold", DoubleValue(threshold - 5));

  Ptr<YansWifiChannel> chan = wifiChannel.Create();
  // switch(mod){
  //   case 1: Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
  //           break;
  //   case 2: Ptr<TwoRayGroundPropagationLossModel> lossModel = CreateObject<TwoRayGroundPropagationLossModel>();
  //           break;
  //   case 3: Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
  //           break;
  // }
  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
  // lossModel -> SetFrequency(2.4e+9);
  // lossModel -> SetHeightAboveZ(1);
  chan -> SetPropagationLossModel(lossModel);

  wifiPhy.SetChannel(chan);

  NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);
  NetDeviceContainer apDevices = wifi.Install(wifiPhy, wifiMac, apNode);

  MobilityHelper mobilityAdhoc;

  ObjectFactory pos;
  pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
  std::stringstream index1, index2;
  index2<<area/2;
  index1<<area/2*(-1);

  pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=" + index1.str() + "|" + "Max=" + index2.str() + "]"));
  pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=" + index1.str() + "|" + "Max=" + index2.str() + "]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

  mobilityAdhoc.SetPositionAllocator ("ns3::RandomWaypointMobilityModel",
                                 "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=12]"),
                                 "Pause", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=2]"),
                                 "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator(taPositionAlloc);
  mobilityAdhoc.Install(adhocNodes);

  mobilityAdhoc.Install(adhocNodes);//????

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc -> Add(Vector (0.0,0.0,0.0));
  MobilityHelper apMobility;
  apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  apMobility.SetPositionAllocator(positionAlloc);
  apMobility.Install(apNode);

  InternetStackHelper internet;
  internet.Install (apNode);
  internet.Install (adhocNodes);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  
  Ipv4InterfaceContainer apInterfaces;
  apInterfaces = ipv4.Assign(apDevices);

  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = ipv4.Assign(adhocDevices);

  ApplicationContainer cbrApps;
  uint16_t cbrPort = 12345;
  OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (apInterfaces.GetAddress(0), cbrPort));
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (256));
  onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

  // flow 1:  node 0 -> node n - 1
  for(int i = 0; i < nodes - 1; i++)
  {
    onOffHelper.SetAttribute ("DataRate", StringValue ("409600bps"));
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
    onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1+0.001*i)));
    onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds (101+0.001*i)));
    cbrApps.Add (onOffHelper.Install (adhocNodes.Get (i)));
  }
   
  // 8. Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  // 9. Run simulation for 10 seconds
  Simulator::Stop (Seconds (150));
  Simulator::Run ();

  // 10. Print per flow statistics
  long int sendPackets = 0;
  long int receivePackets = 0;
  double Throughput = 0;
  int zero = 0;

  std::ofstream fout;
  fout.open("data1.txt", std::ios::app);
  
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
    if (i->second.rxPackets == 0)
      zero ++;
    sendPackets += i->second.txPackets;
    receivePackets += i->second.rxPackets;
  }
  std::cout << sendPackets <<"\n";
  Throughput = float(receivePackets) / sendPackets;
  fout << sendPackets << "  " << receivePackets << "  " << zero << "  " << Throughput << " Area: " << area <<" nodes: "<< nodes <<"\n";
  fout.close();
  // 11. Cleanup
  Simulator::Destroy ();
}

int main (int argc, char **argv)
{
  for(int n = 0; n < 10; n++){
    for(int nodes = 50; nodes < 202; nodes += nodes){
      for(int area = 100; area < 251; area += 150){
        std::cout << "------------------------------------------------\n";
        std::cout<<nodes<<"\n";
        experiment (true, nodes, area);
      }
    }
  }
  return 0;
}
