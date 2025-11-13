/*
 * Copyright (c) 2024 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: zubow@tkn.tu-berlin.de
 */

#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"
#include "lib/sionna-utils.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-psdu.h" // For WifiPsdu
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-phy.h"
#include <map>

/**
 * An example showing the usage of ns3sionna together with the YansWiFiChannel. Here Sionna is used
 * to compute the wideband pathloss only (no fast fading is considered).
 * An indoor scenario with static AP and mobile STA using 80 MHz channel is simulated.
 * STA sends packets to the AP from which the SNR is computed and plotted.
 *
 * Limitations: only SISO so far
 *
 *  To run: ./example-sionna-sensing-mobile.sh
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleMobilitySionna");

std::map<Ipv4Address, uint32_t> g_ipToNodeIdMap;  // Global map

// time, node_id, snr, position
std::string csiFname = "example-mobility-sionna-snr.csv";
std::ofstream ofs_csi(csiFname);

void BuildIpToNodeIdMap ()
{
    g_ipToNodeIdMap.clear ();
    for (uint32_t i = 0; i < NodeList::GetNNodes (); ++i)
    {
        Ptr<Node> node = NodeList::GetNode (i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
        if (ipv4)
        {
            uint32_t nInterfaces = ipv4->GetNInterfaces ();
            for (uint32_t j = 0; j < nInterfaces; ++j)
            {
                uint32_t nAddrs = ipv4->GetNAddresses (j);
                for (uint32_t k = 0; k < nAddrs; ++k)
                {
                    Ipv4Address addr = ipv4->GetAddress (j, k).GetLocal ();
                    g_ipToNodeIdMap[addr] = node->GetId ();  // Insert (overwrites if duplicate)
                }
            }
        }
    }
    NS_LOG_INFO ("Built IP-to-NodeID map with " << g_ipToNodeIdMap.size () << " entries");
}

// Fast lookup function
uint32_t GetNodeIdFromIpv4Address (Ipv4Address targetAddr)
{
    auto it = g_ipToNodeIdMap.find (targetAddr);
    if (it != g_ipToNodeIdMap.end ())
    {
        return it->second;
    }
    NS_LOG_WARN ("No node found for IPv4 address " << targetAddr);
    return 0xFFFFFFFF;
}


void
PhyRxOkTrace(std::string context, Ptr<const Packet> p, double snr, WifiMode mode, WifiPreamble preamble)
{
    double snrDb = 10 * std::log10(snr);

    uint32_t node_id = ContextToNodeId(context);
    // get position
    Ptr<Node> node = NodeList::GetNode(node_id);
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Node: " << node->GetId()
        << ": Pos: (" << pos.x << "," << pos.y << "," << pos.z << ")");

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: PHY-RX-OK node="
        << node_id << " size=" << p->GetSize() << " snr=" << snrDb
        << "db, mode=" << mode << " preamble=" << preamble);

    ofs_csi << Simulator::Now().GetSeconds() << "," << node_id << "," << snrDb << "," << pos.x << "," << pos.y << "," << pos.z << std::endl;
    ofs_csi << std::flush;
}

void
TracePacketReception(std::string context, Ptr<const Packet> p, uint16_t channelFreqMhz,
                     WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
	NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Trace: nodeId=" << staId << ", signal="
        << signalNoise.signal << "dBm " << "noise=" << signalNoise.noise << "dBm");
}

void
RunSimulation(SionnaHelper &sionnaHelper,
              const bool caching,
              const uint32_t seed,
              const int wifi_channel_num,
              const int channel_width,
              const bool verbose)
{
    NS_LOG_INFO("New simulation with seed " << seed);
    RngSeedManager::SetSeed(seed);

    NodeContainer wifiStaNode;
    wifiStaNode.Create(1);

    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();

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

    WifiStandard wifi_standard = WIFI_STANDARD_80211ax;
    wifi.SetStandard(wifi_standard);

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " + std::to_string(channel_width) + ", BAND_5GHZ, 0}";
    phy.Set("ChannelSettings", StringValue(channelStr));
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    staDevices = wifi.Install(phy, mac, wifiStaNode);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true),
        "BeaconInterval", TimeValue(Seconds(5.120)), "EnableBeaconJitter", BooleanValue(false));
    apDevices = wifi.Install(phy, mac, wifiApNode);

    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    mobility.Install(wifiApNode);

    mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                              "Model",
                              EnumValue(SionnaMobilityModel::MODEL_RANDOM_WALK),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "Distance",
                              DoubleValue(6));
    mobility.Install(wifiStaNode);

    wifiStaNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(4.0, 2.0, 1.0));
    wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1.0, 2.0, 1.0));

    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNode);

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiStaInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer wifiApInterfaces = address.Assign(apDevices);

    BuildIpToNodeIdMap();  // Build once here

    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(wifiStaNode);
    serverApps.Start(Seconds(0.5));
    serverApps.Stop(Seconds(200.0));

    Config::Connect(
      "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/MonitorSnifferRx",
      MakeCallback(&TracePacketReception));

    // Trace PHY Rx success events
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/RxOk",
                    MakeCallback(&PhyRxOkTrace));

    UdpEchoClientHelper echoClient(Ipv4Address ("255.255.255.255"), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(50001));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(25)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiApNode);
    clientApps.Start(Seconds(0.6));
    clientApps.Stop(Seconds(200.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // set center frequency & bandwidth for Sionna
    double channelWidth = get_channel_width(apDevices.Get(0));
    sionnaHelper.Configure(get_center_freq(apDevices.Get(0)),
        channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));

    Simulator::Stop(Seconds(200.0));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    ofs_csi.close();
    std::cout << "Trace results can be found in: " << csiFname << std::endl;
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool caching = true;
    std::string environment = "2_rooms_with_door/2_rooms_with_door_open.xml";
    int wifi_channel_num = 44;
    int channel_width = 20;
    uint32_t numseeds = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
    cmd.AddValue("environment", "Xml file of environment", environment);
    cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
    cmd.AddValue("channelWidth", "The WiFi channel width in MHz", channel_width);
    cmd.AddValue("numseeds", "Number of seeds", numseeds);
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
        LogComponentEnable("ExampleMobilitySionna", LOG_INFO);
    }

    NS_LOG_INFO("1 ap and 1 moving sta scenario with sionna");

    std::string server_url = "tcp://localhost:5555";
    SionnaHelper sionnaHelper(environment, server_url);

    for (uint32_t seed = 1; seed <= numseeds; seed++)
    {
        RunSimulation(sionnaHelper, caching, seed, wifi_channel_num, channel_width, verbose);
    }

    sionnaHelper.Destroy();

    return 0;
}
