/*
 * Copyright (c) 2024 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 */

#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"
#include "lib/sionna-spectrum-propagation-loss-model.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ssid.h"
#include "ns3/spectrum-module.h"
#include "ns3/spectrum-wifi-helper.h"


/**
 * Advanced example showing an outdoor scenario with static AP and mobile STA (v=7m/s).
 *
 * Limitations: only SISO so far
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleMunichMobilitySionna");

// mapping of IPv4 addr to nodeIds
std::map<Ipv4Address, uint32_t> g_ipToNodeIdMap;

std::string prefix = "example-munich-mobility-sionna";
// Exporting CSI data
std::string csiFname = prefix + "-csi.csv";
std::ofstream ofs_csi(csiFname);
// Exporting pathloss data
std::string plFname = prefix + "-pathloss.csv";
std::ofstream ofs_pl(plFname);
// Exporting time and RX (STA) node location
std::string tpFname = prefix + "-time-pos.csv";
std::ofstream ofs_tp(tpFname);

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
                    if (!addr.IsLocalhost())
                    {
                        NS_LOG_DEBUG("Node Id: " << node->GetId () << ", IP: " << addr);
                        g_ipToNodeIdMap[addr] = node->GetId ();  // Insert (overwrites if duplicate)
                    }
                }
            }
        }
    }
    NS_LOG_DEBUG ("Built IP-to-NodeID map with " << g_ipToNodeIdMap.size () << " entries");
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

/*
 * Trace application layer, here the UdpEchoServer.
 * For each the received packet the CFR (CSI) is retrieved from the tag and dumped to file.
 */
void RxTraceWithAddresses(std::string context, Ptr<const Packet> packet, const Address &from, const Address &to) {

    // Lookup ID of transmitter & receiver
    uint32_t src_nodeId = GetNodeIdFromIpv4Address(InetSocketAddress::ConvertFrom(from).GetIpv4());
    Ptr<Node> src_node = NodeList::GetNode(src_nodeId);
    Ptr<MobilityModel> mobility = src_node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Node: " << src_node->GetId() << ": Pos: (" << pos.x << "," << pos.y << "," << pos.z << ")");

    NS_LOG_INFO("*** " << Simulator::Now().GetSeconds() << "s [" << context << "]: Server received packet of " << packet->GetSize() << " bytes"
        << " from: " << InetSocketAddress::ConvertFrom(from).GetIpv4() << "(" << src_nodeId << ") port "
        << " to: " << InetSocketAddress::ConvertFrom(to).GetIpv4() << "(/) port " << InetSocketAddress::ConvertFrom(to).GetPort());

    // check to see whether packet is tagged with CSI/CFR
    CFRTag tag;
    if (packet->PeekPacketTag(tag))
    {
        // dump CSI
        dumpComplexVecToStream(tag.GetComplexes(), ofs_csi); ofs_csi << std::flush;
        // dump pathloss
        double pathLossDb = tag.GetPathloss();
        ofs_pl << pathLossDb << std::endl; ofs_pl << std::flush;
        // dump rx node position
        ofs_tp << Simulator::Now().GetSeconds() << "," << pos.x << "," << pos.y << "," << pos.z << std::endl;
        ofs_tp << std::flush;
    }
}

void
RunSimulation(SionnaHelper &sionnaHelper,
              const bool caching,
              const uint32_t seed,
              const int wifi_channel_num,
              const int channel_width,
              const double sim_duration_sec,
              const bool verbose)
{
    std::cout << "New simulation for Tmax=" << sim_duration_sec << "sec" << std::endl;
    RngSeedManager::SetSeed(seed);

    NodeContainer wifiStaNode;
    wifiStaNode.Create(1);

    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetSionnaHelper(sionnaHelper);
    propagationCache->SetCaching(caching);

    std::cout << "Using spectrum model" << std::endl;
    Ptr<MultiModelSpectrumChannel> spectrumChannel =
        CreateObject<MultiModelSpectrumChannel>();

    Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
    lossModel->SetPropagationCache(propagationCache);

    spectrumChannel->AddPropagationLossModel(lossModel);

    Ptr<SionnaSpectrumPropagationLossModel> spectrumLossModel = CreateObject<SionnaSpectrumPropagationLossModel>();
    spectrumLossModel->SetPropagationCache(propagationCache);

    spectrumChannel->AddSpectrumPropagationLossModel(spectrumLossModel);

    Ptr<SionnaPropagationDelayModel> delayModel = CreateObject<SionnaPropagationDelayModel>();
    delayModel->SetPropagationCache(propagationCache);
    spectrumChannel->SetPropagationDelayModel(delayModel);

    SpectrumWifiPhyHelper spectrumPhy;
    spectrumPhy.SetChannel(spectrumChannel);
    spectrumPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    spectrumPhy.Set("TxPowerStart", DoubleValue(20));
    spectrumPhy.Set("TxPowerEnd", DoubleValue(20));

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    WifiHelper wifi;

    WifiStandard wifi_standard = WIFI_STANDARD_80211ax; // WIFI6
    wifi.SetStandard(wifi_standard);

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " + std::to_string(channel_width) + ", BAND_5GHZ, 0}";

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    //wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");
    staDevices = wifi.Install(spectrumPhy, mac, wifiStaNode);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true),
        "BeaconInterval", TimeValue(Seconds(5.120)), "EnableBeaconJitter", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    apDevices = wifi.Install(spectrumPhy, mac, wifiApNode);

    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    mobility.Install(wifiApNode);

    mobility.SetMobilityModel("ns3::SionnaMobilityModel",
                              "Model",
                              EnumValue(SionnaMobilityModel::MODEL_RANDOM_WALK),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=7.0]"), // m/s
                              "Distance",
                              DoubleValue(50));
    mobility.Install(wifiStaNode);

    wifiStaNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(45.0, 90.0, 1.5));
    wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(8.5, 21.0, 27.0));

    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNode);

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiStaInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer wifiApInterfaces = address.Assign(apDevices);

    BuildIpToNodeIdMap();  // Build once here


    // Set up applications
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(wifiApNode);
    serverApps.Start(Seconds(0.5));
    serverApps.Stop(Seconds(sim_duration_sec));

    // App layer tracing of RX events to capture CSI
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/RxWithAddresses",
                    MakeCallback(&RxTraceWithAddresses));

    Ipv4Address wifi_ip_addr = wifiApInterfaces.GetAddress(0);
    //std::cout << "AP IP: " << wifi_ip_addr << std::endl;

    UdpEchoClientHelper echoClient(wifi_ip_addr, 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(10000));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNode);
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(sim_duration_sec));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // set center frequency & bandwidth for Sionna
    double channelWidth = get_channel_width(apDevices.Get(0));
    sionnaHelper.Configure(get_center_freq(apDevices.Get(0)),
        channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));

    Simulator::Stop(Seconds(sim_duration_sec));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    ofs_csi.close();
    ofs_pl.close();
    ofs_tp.close();
    std::cout << "CSI/pathloss/time/pos results can be found in: " << prefix << "*.csv" << std::endl;
    std::cout << "For plotting run: python plot3d_munich.py " << std::endl;

    std::cout << std::endl << std::endl;
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool caching = true;
    std::string environment = "munich/munich.xml";
    int wifi_channel_num = 42; // 40
    int channel_width = 80; // 20
    uint32_t numseeds = 1;
    double sim_duration_sec = 100.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
    cmd.AddValue("environment", "Xml file of environment", environment);
    cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
    cmd.AddValue("channelWidth", "The WiFi channel width in MHz", channel_width);
    cmd.AddValue("numseeds", "Number of seeds", numseeds);
    cmd.AddValue("simDurationSec", "Simulation duration in sec", sim_duration_sec);
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

    std::cout << "1 ap and 1 moving sta scenario with sionna" << std::endl << std::endl;
    std::cout << "Config: CH=" << wifi_channel_num << ",BW=" << channel_width << std::endl;

    std::string server_url = "tcp://localhost:5555";
    SionnaHelper sionnaHelper(environment, server_url);

    for (uint32_t seed = 1; seed <= numseeds; seed++)
    {
        RunSimulation(sionnaHelper, caching, seed, wifi_channel_num, channel_width, sim_duration_sec, verbose);
    }

    sionnaHelper.Destroy();

    return 0;
}
