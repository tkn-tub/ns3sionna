/*
 * Copyright (c) 2024 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Zubow
 */

// Sionna models
#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"

// Ns-3 modules
#include "../../src/wifi/model/yans-wifi-phy.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleSionna");

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool tracing = true;
    bool caching = true;
    bool optimizer = true;
    std::string environment = "free_space/free_space.xml";
    int wifi_channel_num = 6;
    int channel_width = 20; // 802.11g supports only 20MHz
    double dist_ap_sta = 300.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
    cmd.AddValue("optimizer", "Enable optimizer of propagation delay and loss", optimizer);
    cmd.AddValue("environment", "Xml file of environment", environment);
    cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
    cmd.AddValue("distApSta", "The WiFi channel number", dist_ap_sta);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        LogComponentEnable("YansWifiChannel", LOG_DEBUG);
        LogComponentEnable("YansWifiChannel", LOG_PREFIX_TIME);
        LogComponentEnable("SionnaPropagationDelayModel", LOG_INFO);
        LogComponentEnable("SionnaPropagationCache", LOG_INFO);
    }

    std::cout << "Example scenario with sionna" << std::endl << std::endl;

    SionnaHelper sionnaHelper(environment, "tcp://localhost:5555");

    // Create nodes
    NodeContainer wifiStaNode;
    wifiStaNode.Create(1);

    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    // Create a channel
    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetSionnaHelper(sionnaHelper);
    propagationCache->SetCaching(caching);
    propagationCache->SetOptimize(optimizer);

    Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();
    delayModel->SetPropagationCache(propagationCache);

    Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
    lossModel->SetPropagationCache(propagationCache);

    channel->SetPropagationLossModel(lossModel);
    channel->SetPropagationDelayModel(delayModel);

    // WiFi configuration
    YansWifiPhyHelper phy;
    phy.SetChannel(channel);

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    WifiHelper wifi;

    WifiStandard wifi_standard = WIFI_STANDARD_80211g;
    wifi.SetStandard(wifi_standard);

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " + std::to_string(channel_width) + ", BAND_2_4GHZ, 0}";
    phy.Set("ChannelSettings", StringValue(channelStr));

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    staDevices = wifi.Install(phy, mac, wifiStaNode);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true), "BeaconInterval", TimeValue(Seconds(5.120)), "EnableBeaconJitter", BooleanValue(false));
    apDevices = wifi.Install(phy, mac, wifiApNode);

    // Mobility configuration
    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    mobility.Install(wifiStaNode);
    mobility.Install(wifiApNode);

    wifiStaNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(dist_ap_sta, 0.0, 0.0));
    wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));

    // Set up Internet stack and assign IP addresses
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNode);

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiStaInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer wifiApInterfaces = address.Assign(apDevices);

    // Set up applications
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(wifiApNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpEchoClientHelper echoClient(wifiApInterfaces.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(2));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNode);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // set center frequency & bandwidth for Sionna
    double channelWidth = get_channel_width(apDevices.Get(0));
    sionnaHelper.Configure(get_center_freq(apDevices.Get(0)),
        channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));

    // Tracing
    if (tracing)
    {
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.EnablePcap("example-sionna", apDevices.Get(0));
        phy.EnablePcap("example-sionna", staDevices.Get(0));
    }

    if (verbose)
    {
        // Print node information
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

    // Simulation
    Simulator::Stop(Seconds(5.0));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    sionnaHelper.Destroy();

    return 0;
}
