/*
* Copyright (c) 2024 Yannik Pilz
*
* SPDX-License-Identifier: GPL-2.0-only
*
* Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
*/

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"
#include "../../src/wifi/model/yans-wifi-phy.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PerformanceNs3");

double get_center_freq(Ptr<NetDevice> nd)
{
    Ptr<WifiPhy> wp = nd->GetObject<WifiNetDevice>()->GetPhy();
    return wp->GetObject<YansWifiPhy>()->GetFrequency();
}

double
RunSimulation(const uint32_t numStas, const int channel_no, const bool mobile_scenario,
              const double mobile_speed, const int udp_pkt_interval, const bool verbose)
{
   Ptr<Building> b = CreateObject<Building>();
   b->SetBoundaries(Box(0.0, 6.0, 0.0, 4.0, 0.0, 2.5));
   b->SetBuildingType(Building::Office);
   b->SetExtWallsType(Building::StoneBlocks);
   b->SetNFloors(1);
   b->SetNRoomsX(1);
   b->SetNRoomsY(1);

   NodeContainer wifiStaNodes;
   wifiStaNodes.Create(numStas);

   NodeContainer wifiApNode;
   wifiApNode.Create(1);

   Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
   Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
   Ptr<HybridBuildingsPropagationLossModel> lossModel = CreateObject<HybridBuildingsPropagationLossModel>();
   //lossModel->SetFrequency(get_center_freq(apDevices.Get(0)));
   channel->SetPropagationLossModel(lossModel);
   channel->SetPropagationDelayModel(delayModel);

   YansWifiPhyHelper phy;
   phy.SetChannel(channel);

   WifiMacHelper mac;
   Ssid ssid = Ssid("ns-3-ssid");

   WifiHelper wifi;

   WifiStandard wifi_standard = WIFI_STANDARD_80211g;
   wifi.SetStandard(wifi_standard);

   std::string channelStr = "{" + std::to_string(channel_no) + ", " + std::to_string(20) + ", BAND_2_4GHZ, 0}";
   phy.Set("ChannelSettings", StringValue(channelStr));

   NetDeviceContainer staDevices;
   mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
   staDevices = wifi.Install(phy, mac, wifiStaNodes);

   NetDeviceContainer apDevices;
   mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true));
   apDevices = wifi.Install(phy, mac, wifiApNode);

   lossModel->SetFrequency(get_center_freq(apDevices.Get(0)));

   MobilityHelper mobility;

   if (mobile_scenario)
   {
       mobility.Install(wifiApNode);

       mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                 "Bounds",
                                 RectangleValue(Rectangle(0.0, 6.0, 0.0, 4.0)),
                                 "Speed",
                                 StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(mobile_speed) + "]"));
       mobility.Install(wifiStaNodes);
   }
   else
   {
       mobility.Install(wifiStaNodes);
       mobility.Install(wifiApNode);
   }

   BuildingsHelper::Install(wifiStaNodes);
   BuildingsHelper::Install(wifiApNode);

   wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1.0, 2.0, 1.0));

   Ptr<UniformRandomVariable> randX = CreateObject<UniformRandomVariable>();
   Ptr<UniformRandomVariable> randY = CreateObject<UniformRandomVariable>();
   randX->SetAttribute("Min", DoubleValue(0.1));
   randX->SetAttribute("Max", DoubleValue(5.9));
   randY->SetAttribute("Min", DoubleValue(0.1));
   randY->SetAttribute("Max", DoubleValue(3.9));
   for (uint32_t i = 0; i < numStas; i++)
   {
       wifiStaNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(randX->GetValue(), randY->GetValue(), 1.0));
   }

   InternetStackHelper stack;
   stack.Install(wifiApNode);
   stack.Install(wifiStaNodes);

   Ipv4AddressHelper address;

   address.SetBase("10.1.1.0", "255.255.255.0");
   Ipv4InterfaceContainer wifiStaInterfaces = address.Assign(staDevices);
   Ipv4InterfaceContainer wifiApInterfaces = address.Assign(apDevices);

   UdpEchoServerHelper echoServer(9);

   ApplicationContainer serverApps = echoServer.Install(wifiStaNodes);
   serverApps.Start(Seconds(0.9));
   serverApps.Stop(Seconds(10.0));

   UdpEchoClientHelper echoClient(Ipv4Address ("255.255.255.255"), 9);
   echoClient.SetAttribute("MaxPackets", UintegerValue(1e9));
   echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(udp_pkt_interval)));
   echoClient.SetAttribute("PacketSize", UintegerValue(100));

   ApplicationContainer clientApps = echoClient.Install(wifiApNode);
   clientApps.Start(Seconds(1.0));
   clientApps.Stop(Seconds(10.0));

   Ipv4GlobalRoutingHelper::PopulateRoutingTables();

   if (verbose)
   {
       std::cout << "----------Node Information----------" << std::endl;
       NodeContainer c = NodeContainer::GetGlobal();
       for (auto iter = c.Begin(); iter != c.End(); ++iter)
       {
           std::cout << "NodeID: " << (*iter)->GetId() << ", ";

           Ptr<MobilityModel> mobilityModel = (*iter)->GetObject<MobilityModel>();
           if (mobilityModel)
           {
               std::cout << mobilityModel->GetInstanceTypeId().GetName() << " (";
               Vector position = mobilityModel->GetPosition();
               Vector velocity = mobilityModel->GetVelocity();
               std::cout << "Pos: [" << position.x << ", " << position.y << ", " << position.z << "]" << ", ";
               std::cout << "Vel: [" << velocity.x << ", " << velocity.y << ", " << velocity.z << "])" << std::endl;
           }
           else
           {
               std::cout << "No MobilityModel" << std::endl;
           }
       }
   }

   Simulator::Stop(Seconds(10.0));

   auto startTime = std::chrono::system_clock::now();

   Simulator::Run();
   Simulator::Destroy();

   auto endTime = std::chrono::system_clock::now();
   std::chrono::duration<double> elapsedTime = endTime - startTime;
   double computationTime = elapsedTime.count();
   std::cout << "Finished simulation with " << numStas << " stations in " << computationTime << " sec";

   std::cout << std::endl << std::endl;
   return computationTime;
}

int
main(int argc, char* argv[])
{
   bool verbose = false;
   int wifi_channel_num = 1;
   bool mobile_scenario = false;
   double mobile_speed = 1.0;
   int udp_pkt_interval = 1;
   int sim_max_stas = 1;

   CommandLine cmd(__FILE__);
   cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
   cmd.AddValue("mobile_scenario", "Enable node movement", mobile_scenario);
   cmd.AddValue("mobile_speed", "STA speed when mobile_scenario is true", mobile_speed);
   cmd.AddValue("udp_pkt_interval", "UDP packet interval (in ms) used by STAs", udp_pkt_interval);
   cmd.AddValue("sim_max_stas", "Max number of STAs to be simulated", sim_max_stas);
   cmd.AddValue("verbose", "Enable logging", verbose);
   cmd.Parse(argc, argv);

   if (verbose)
   {
       LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
       LogComponentEnable("UdpEchoClientApplication", LOG_PREFIX_TIME);
       LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
       LogComponentEnable("UdpEchoServerApplication", LOG_PREFIX_TIME);
       LogComponentEnable("YansWifiChannel", LOG_DEBUG);
       LogComponentEnable("YansWifiChannel", LOG_PREFIX_TIME);
       LogComponentEnable("BuildingsPropagationLossModel", LOG_INFO);
       LogComponentEnable("BuildingsPropagationLossModel", LOG_PREFIX_TIME);
       LogComponentEnable("ItuR1238PropagationLossModel", LOG_INFO);
       LogComponentEnable("ItuR1238PropagationLossModel", LOG_PREFIX_TIME);
   }

   std::cout << "Performance test: 1 AP and N STAs with pure ns3" << std::endl;
   std::cout << "Config: ch " << wifi_channel_num << " mob " << mobile_scenario;
   std::cout << " speed " << mobile_speed << " pktinterval " << udp_pkt_interval << std::endl;

   for (uint32_t numStas = 1; numStas <= (uint32_t)sim_max_stas; numStas = numStas * 2)
   {
       RunSimulation(numStas, wifi_channel_num, mobile_scenario, mobile_speed, udp_pkt_interval, verbose);
   }

   return 0;
}
