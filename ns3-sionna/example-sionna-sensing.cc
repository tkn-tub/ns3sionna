/*
* Copyright (c) 2024 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: zubow@tkn.tu-berlin.de
 */

// Sionna models
#include "lib/sionna-helper.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"
#include "lib/sionna-spectrum-propagation-loss-model.h"

// Ns-3 modules
#include <ns3/wifi-spectrum-phy-interface.h>
#include "lib/cfr-tag.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/spectrum-module.h"


/**
 * Simple example showing the use of MultiModelSpectrumChannel with ns3sionna.
 * Scenario: single AP with two connected STAs operating on 80 MHz channel in room scenario.
 * All WiFi nodes are static and each STA sends a single packet from which the CSI is retrieved
 * and stored in a file.
 * Note: due to fully static configuration the channel is only computed once.
 *
 * Limitations: only SISO so far
 *
 * To run: ./example-sionna-sensing.sh
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleSionnaSensing");

// mapping of IPv4 addr to nodeIds
std::map<Ipv4Address, uint32_t> g_ipToNodeIdMap;

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

    // Lookup ID of transmitter
    uint32_t src_nodeId = GetNodeIdFromIpv4Address(InetSocketAddress::ConvertFrom(from).GetIpv4());
    Ptr<Node> src_node = NodeList::GetNode(src_nodeId);
    Ptr<MobilityModel> mobility = src_node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: " << src_node->GetId() << ": (" << pos.x << "," << pos.y << "," << pos.z << ")");

    NS_LOG_INFO("*** " << Simulator::Now().GetSeconds() << "s [" << context << "]: Server received packet of " << packet->GetSize() << " bytes"
        << " from: " << InetSocketAddress::ConvertFrom(from).GetIpv4() << "(" << src_nodeId << ") port "
        << " to: " << InetSocketAddress::ConvertFrom(to).GetIpv4() << "(/) port " << InetSocketAddress::ConvertFrom(to).GetPort());

    // check to see whether packet is tagged with CSI/CFR
    CFRTag tag;
    if (packet->PeekPacketTag(tag))
    {
        //double pathLossDb = tag.GetPathloss();
        dumpComplexVecToFile(tag.GetComplexes(), "csi_node" + std::to_string(src_nodeId) + ".csv");
    }
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool tracing = true;
    bool caching = true;
    //std::string environment = "simple_room/simple_room.xml";
    std::string environment = "2_rooms_with_door/2_rooms_with_door_open.xml";
    int wifi_channel_num = 42; // center at 5210
    int app_max_packets = 1;
    int channelWidth = 80;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
    cmd.AddValue("environment", "Xml file of environment", environment);
    cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
    cmd.AddValue("appMaxPackets", "The maximum number of packets transmitted by app", app_max_packets);
    cmd.AddValue("channelWidth", "The WiFi channel width in MHz", channelWidth);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("ExampleSionnaSensing", LOG_INFO);
        LogComponentEnable("SionnaPropagationDelayModel", LOG_INFO);
        LogComponentEnable("SionnaPropagationLossModel", LOG_INFO);
        LogComponentEnable("SionnaPropagationCache", LOG_INFO);
        LogComponentEnable("SionnaSpectrumPropagationLossModel", LOG_INFO);
    }

    std::cout << "Example spectrum model wifi scenario with sionna" << std::endl << std::endl;

    SionnaHelper sionnaHelper(environment, "tcp://localhost:5555");

    // Create nodes
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(2);

    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetSionnaHelper(sionnaHelper);
    propagationCache->SetCaching(caching);

    // new
    Ptr<MultiModelSpectrumChannel> spectrumChannel =
        CreateObject<MultiModelSpectrumChannel>();

    Ptr<SionnaPropagationLossModel> lossModel = CreateObject<SionnaPropagationLossModel>();
    lossModel->SetPropagationCache(propagationCache);

    spectrumChannel->AddPropagationLossModel(lossModel);

    // SISO only
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

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    staDevices = wifi.Install(spectrumPhy, mac, wifiStaNodes);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true),
        "BeaconInterval", TimeValue(Seconds(5.120)), "EnableBeaconJitter", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    apDevices = wifi.Install(spectrumPhy, mac, wifiApNode);

    // Mobility configuration: fixed nodes
    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    mobility.Install(wifiStaNodes);
    mobility.Install(wifiApNode);

    wifiStaNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(5.0, 2.0, 1.0));
    wifiStaNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(2.0, 3.0, 1.0));
    wifiApNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(1.0, 2.0, 1.0));

    // Set up Internet stack and assign IP addresses
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiStaInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer wifiApInterfaces = address.Assign(apDevices);

    BuildIpToNodeIdMap();

    // Set up applications
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(wifiApNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    // App layer tracing of RX events to capture CSI
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/RxWithAddresses",
                    MakeCallback(&RxTraceWithAddresses));

    Ipv4Address wifi_ip_addr = wifiApInterfaces.GetAddress(0);

    UdpEchoClientHelper echoClient(wifi_ip_addr, 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(app_max_packets));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNodes);
    clientApps.Start(Seconds(1.0));
    clientApps.Stop(Seconds(10.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // set center frequency, bandwidth, FFT for Sionna
    double fc = get_center_freq(apDevices.Get(0));
    sionnaHelper.Configure(fc, channelWidth,
        getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));
    sionnaHelper.SetMode(SionnaHelper::MODE_P2P); // compute CSI for P2P only; no look-ahead computation

    if (tracing) // Tracing
    {
        std::cout << "Writing pcap files ..." << std::endl;
        spectrumPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        spectrumPhy.EnablePcap("example-sionna-sensing", apDevices.Get(0));
        spectrumPhy.EnablePcap("example-sionna-sensing", staDevices.Get(0));
        spectrumPhy.EnablePcap("example-sionna-sensing", staDevices.Get(1));
    }

    // Simulation end
    Simulator::Stop(Seconds(2));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    propagationCache->PrintStats();
    sionnaHelper.Destroy();

    return 0;
}

