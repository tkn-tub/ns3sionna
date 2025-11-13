/*
* Copyright (c) 2024 Yannik Pilz, Zubow
*
* SPDX-License-Identifier: GPL-2.0-only
*
* Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
*/

#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"
#include "../../src/wifi/model/yans-wifi-phy.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PerformanceSionna");

double
RunSimulation(const std::string environment, const uint32_t numStas, const int channel_no, const bool mobile_scenario,
              const double mobile_speed, const int udp_pkt_interval, const bool caching,
              const int mode, const int sub_mode, const bool verbose)
{
   NodeContainer wifiStaNodes;
   wifiStaNodes.Create(numStas);

   NodeContainer wifiApNode;
   wifiApNode.Create(1);

   Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();

   std::string server_url = "tcp://localhost:5555";
   SionnaHelper sionnaHelper(environment, server_url);

   Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
   propagationCache->SetSionnaHelper(sionnaHelper);
   propagationCache->SetCaching(caching);

   Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();
   delayModel->SetPropagationCache(propagationCache);

   Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
   lossModel->SetPropagationCache(propagationCache);

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

   MobilityHelper mobility;

   if (mobile_scenario)
   {
       mobility.SetMobilityModel("ns3::SionnaMobilityModel");
       mobility.Install(wifiApNode);

       mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                                 "Model",
                                 EnumValue(SionnaMobilityModel::MODEL_RANDOM_WALK),
                                 "Speed",
                                 StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(mobile_speed) + "]"));
       mobility.Install(wifiStaNodes);
   }
   else
   {
       mobility.SetMobilityModel("ns3::SionnaMobilityModel");
       mobility.Install(wifiStaNodes);
       mobility.Install(wifiApNode);
   }

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

   // set center frequency & bandwidth for Sionna
    double channelWidth = get_channel_width(apDevices.Get(0));
   sionnaHelper.Configure(get_center_freq(apDevices.Get(0)),
       channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));
   sionnaHelper.SetMode(mode);
   sionnaHelper.SetSubMode(sub_mode);

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
               std::cout << "Vel: [" << velocity.x << ", " << velocity.y << ", " << velocity.z << "]";

               Ptr<SionnaMobilityModel> sionnaMobilityModel = DynamicCast<SionnaMobilityModel>(mobilityModel);
               if (sionnaMobilityModel)
               {
                   std::cout << ", " << "Model: " << sionnaMobilityModel->GetModel() << ", ";
                   std::cout << "Mode: " << sionnaMobilityModel->GetMode() << ", ";
                   std::cout << "ModeTime: " << sionnaMobilityModel->GetModeTime().GetSeconds() << ", ";
                   std::cout << "ModeDistance: " << sionnaMobilityModel->GetModeDistance() << ", ";
                   std::cout << "Speed: " << sionnaMobilityModel->GetSpeed()->GetInstanceTypeId().GetName() << ", ";
                   std::cout << "Direction: " << sionnaMobilityModel->GetDirection()->GetInstanceTypeId().GetName();
               }

               std::cout << ")" << std::endl;
           }
           else
           {
               std::cout << "No MobilityModel" << std::endl;
           }
       }
   }

   Simulator::Stop(Seconds(10.0));

   auto startTime = std::chrono::system_clock::now();

   sionnaHelper.Start();

   Simulator::Run();
   Simulator::Destroy();

    std::cout << "Ns3-sionna: cache hit ratio: " <<  propagationCache->GetStats() << std::endl;

   sionnaHelper.Destroy();

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
   bool caching = true;
   std::string environment = "simple_room/simple_room.xml";
   int wifi_channel_num = 1;
   bool mobile_scenario = false;
   double mobile_speed = 1.0;
   int udp_pkt_interval = 1;
   int sim_min_stas = 1;
   int sim_max_stas = 1;
   int mode = 3;
   int sub_mode = 16;

   CommandLine cmd(__FILE__);
   cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
   cmd.AddValue("mobile_scenario", "Enable node movement", mobile_scenario);
   cmd.AddValue("mobile_speed", "STA speed when mobile_scenario is true", mobile_speed);
   cmd.AddValue("udp_pkt_interval", "UDP packet interval (in ms) used by STAs", udp_pkt_interval);
   cmd.AddValue("sim_min_stas", "Min number of STAs to be simulated", sim_min_stas);
   cmd.AddValue("sim_max_stas", "Max number of STAs to be simulated", sim_max_stas);
   cmd.AddValue("environment", "Xml file of Sionna environment", environment);
   cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
   cmd.AddValue("mode", "The Sionna mode", mode);
   cmd.AddValue("sub_mode", "The Sionna submode", sub_mode);
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
       LogComponentEnable("SionnaPropagationDelayModel", LOG_INFO);
       LogComponentEnable("SionnaPropagationDelayModel", LOG_PREFIX_TIME);
       LogComponentEnable("SionnaPropagationCache", LOG_INFO);
       LogComponentEnable("SionnaPropagationCache", LOG_PREFIX_TIME);
   }

   std::cout << "Performance test: 1 AP and N STAs with ns3sionna" << std::endl;
   std::cout << "Config: ch " << wifi_channel_num << " mob " << mobile_scenario;
   std::cout << " speed " << mobile_speed << " pktinterval " << udp_pkt_interval;
   std::cout << " caching " << caching << " env " << environment;
   std::cout << " mode " << mode << " submode " << sub_mode << std::endl;

   uint32_t numStas = sim_min_stas;
   double computationTime = 0.0;
   while (computationTime < 2 * 60 * 60 && numStas <= (uint32_t)sim_max_stas) // as long as a single run is below 2h
   {
       computationTime = RunSimulation(environment, numStas, wifi_channel_num, mobile_scenario,
                                       mobile_speed, udp_pkt_interval, caching, mode, sub_mode, verbose);
       numStas = numStas * 2;
   }

   return 0;
}
