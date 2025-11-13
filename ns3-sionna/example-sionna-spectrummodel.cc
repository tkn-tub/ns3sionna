/*
* Copyright (c) 2024 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: zubow@tkn.tu-berlin.de
 */

// Sionna models
#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"

// Ns-3 modules
#include <ns3/spectrum-wifi-phy.h>
#include <ns3/wifi-spectrum-phy-interface.h>

#include "lib/sionna-phased-array-spectrum-propagation-loss-model.h"
#include "lib/sionna-spectrum-propagation-loss-model.h"
#include "lib/cfr-tag.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/network-module.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/yans-wifi-helper.h"
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/spectrum-analyzer.h>
#include "ns3/spectrum-module.h"
#include "ns3/wifi-psdu.h"  // For WifiPsdu

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleSionnaSpectrumModel");

std::map<Ipv4Address, uint32_t> g_ipToNodeIdMap;  // Global map

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

Ptr<const WifiPsdu>
CreatePsduFromPacket (Ptr<const Packet> packet)
{
    WifiMacHeader hdr;
    if (!packet->PeekHeader (hdr)) {
        NS_LOG_WARN ("Cannot create PSDU: No WifiMacHeader.");
        return nullptr;
    }

    // Copy payload after header (or use full packet if header is already included)
    uint32_t headerSize = hdr.GetSerializedSize ();
    Ptr<Packet> payload = packet->CreateFragment (headerSize, packet->GetSize () - headerSize);
    // If header was peeked but not removed, adjust: payload = packet->Copy(); payload->RemoveHeader(hdr);

    // Construct new PSDU (const version via Ptr<const>)
    Ptr<WifiPsdu> psdu = Create<WifiPsdu> (payload, hdr);
    return psdu;  // Implicit upcast to const
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

void RxTraceWithAddresses(std::string context, Ptr<const Packet> packet, const Address &from, const Address &to) {

    // Later lookups are O(1)
    uint32_t src_nodeId = GetNodeIdFromIpv4Address(InetSocketAddress::ConvertFrom(from).GetIpv4());

    std::cout << "*** " << Simulator::Now().GetSeconds() << "s [" << context
    << "]: Server received packet of " << packet->GetSize() << " bytes"
    << " from: " << InetSocketAddress::ConvertFrom(from).GetIpv4() << "(" << src_nodeId << ") port "
    << " to: " << InetSocketAddress::ConvertFrom(to).GetIpv4() << "(/) port "
    << InetSocketAddress::ConvertFrom(to).GetPort() << std::endl;

    CFRTag tag;
    if (packet->PeekPacketTag(tag))
    {
        std::cout << "DRIN" << std::endl;
        tag.Print(std::cout);
        dumpComplexVecToFile(tag.GetComplexes(), "csi_node" + std::to_string(src_nodeId) + ".csv");
    }
    std::cout << "***" << std::endl;
}

void
PhyRxOkTraceAtAP(std::string context,
             Ptr<const Packet> p,
             double snr,
             WifiMode mode,
             WifiPreamble preamble)
{
    double snrDb = 10 * std::log10(snr);

    Ptr<const WifiPsdu> psdu = CreatePsduFromPacket (p);
    uint32_t node_id = ContextToNodeId (context);

    std::cout << "PHY-RX-OK time=" << Simulator::Now().As(Time::S) << " node="
                              << node_id << " size=" << p->GetSize()
                              << " snr=" << snrDb << "db, mode=" << mode
                              << " preamble=" << preamble << std::endl;

    if (psdu) {
        CFRTag tag;
        if (psdu->GetPayload(0)->PeekPacketTag(tag)) {
            NS_LOG_UNCOND("Received PPDU tag");
        } else {
            NS_LOG_UNCOND("No PPDU tag found");
        }

        // Now use PSDU methods
        const WifiMacHeader& fullHdr = psdu->GetHeader (0);
        std::cout << "PSDU A1: " << fullHdr.GetAddr1() << ", A2: " << fullHdr.GetAddr2()
            << ", A3: " << fullHdr.GetAddr3() << std::endl;

        if (preamble == WifiPreamble::WIFI_PREAMBLE_HE_SU)
        {
            std::cout << "HE SU RX" << std::endl;
        }
        //Ptr<const Packet> innerPayload = psdu->GetPayload ();
        // ... process
    }
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool tracing = true;
    bool caching = true;
    std::string environment = "simple_room/simple_room.xml";
    int wifi_channel_num = 42; // center at 5210
    int channelWidth = 80;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable logging", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("caching", "Enable caching of propagation delay and loss", caching);
    cmd.AddValue("environment", "Xml file of environment", environment);
    cmd.AddValue("channel", "The WiFi channel number", wifi_channel_num);
    cmd.AddValue("channelWidth", "The WiFi channel width in MHz", channelWidth);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        LogComponentEnable("YansWifiChannel", LOG_DEBUG);
        LogComponentEnable("YansWifiChannel", LOG_PREFIX_TIME);
        LogComponentEnable("SionnaPropagationDelayModel", LOG_INFO);
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

    // for MIMO
    //Ptr<SionnaPhasedArraySpectrumPropagationLossModel> spectrumLossModel = CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();
    //spectrumLossModel->SetPropagationCache(propagationCache);

    //spectrumChannel->AddPhasedArraySpectrumPropagationLossModel(spectrumLossModel);
    // end MIMO

    // for SISO
    Ptr<SionnaSpectrumPropagationLossModel> spectrumLossModel = CreateObject<SionnaSpectrumPropagationLossModel>();
    spectrumLossModel->SetPropagationCache(propagationCache);

    spectrumChannel->AddSpectrumPropagationLossModel(spectrumLossModel);
    // end SISO

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

    WifiStandard wifi_standard = WIFI_STANDARD_80211ax; // AZu: ac/VHT has incorrect tone plan
    wifi.SetStandard(wifi_standard);

    std::string channelStr = "{" + std::to_string(wifi_channel_num) + ", " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    staDevices = wifi.Install(spectrumPhy, mac, wifiStaNodes);

    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true), "BeaconInterval", TimeValue(Seconds(5.120)), "EnableBeaconJitter", BooleanValue(false));
    spectrumPhy.Set("ChannelSettings", StringValue(channelStr));
    apDevices = wifi.Install(spectrumPhy, mac, wifiApNode);

    // Mobility configuration
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

    BuildIpToNodeIdMap();  // Build once here

    // Set up applications
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(wifiApNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    // Trace PHY Rx success events
    Config::Connect("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/RxOk",
                    MakeCallback(&PhyRxOkTraceAtAP));

    // App layer tracing of RX events
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/RxWithAddresses",
                    MakeCallback(&RxTraceWithAddresses));

    UdpEchoClientHelper echoClient(wifiApInterfaces.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(wifiStaNodes);
    clientApps.Start(Seconds(0.1));
    clientApps.Stop(Seconds(2.0));

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // set center frequency for Sionna
    double fc = get_center_freq(apDevices.Get(0));

    // set center frequency & bandwidth for Sionna
    sionnaHelper.Configure(fc, channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));

    // Tracing
    if (tracing)
    {
        spectrumPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        spectrumPhy.EnablePcap("example-sionna-spectrummodel", apDevices.Get(0));
        spectrumPhy.EnablePcap("example-sionna-spectrummodel", staDevices.Get(0));
        spectrumPhy.EnablePcap("example-sionna-spectrummodel", staDevices.Get(1));
    }

    if (verbose)
    {
        // Print node information
        std::cout << "ns3sionna: mobility configuration" << std::endl;
        NodeContainer c = NodeContainer::GetGlobal();
        for (auto iter = c.Begin(); iter != c.End(); ++iter)
        {
            std::cout << "\t nodeID: " << (*iter)->GetId() << ", ";

            Ptr<MobilityModel> mobilityModel = (*iter)->GetObject<MobilityModel>();
            if (mobilityModel)
            {
                std::cout << mobilityModel->GetInstanceTypeId().GetName() << " (";
                Vector position = mobilityModel->GetPosition();
                Vector velocity = mobilityModel->GetVelocity();
                std::cout << "pos: [" << position.x << ", " << position.y << ", " << position.z << "]" << ", ";
                std::cout << "vel: [" << velocity.x << ", " << velocity.y << ", " << velocity.z << "]";
                
                Ptr<SionnaMobilityModel> sionnaMobilityModel = DynamicCast<SionnaMobilityModel>(mobilityModel);
                if (sionnaMobilityModel)
                {
                    std::cout << ", " << "model: " << sionnaMobilityModel->GetModel() << ", ";
                    std::cout << "mode: " << sionnaMobilityModel->GetMode() << ", ";
                    std::cout << "modetime: " << sionnaMobilityModel->GetModeTime().GetSeconds() << ", ";
                    std::cout << "modefistance: " << sionnaMobilityModel->GetModeDistance() << ", ";
                    std::cout << "speed: " << sionnaMobilityModel->GetSpeed()->GetInstanceTypeId().GetName() << ", ";
                    std::cout << "direction: " << sionnaMobilityModel->GetDirection()->GetInstanceTypeId().GetName();
                }
                std::cout << ")" << std::endl;
            }
            else
            {
                std::cout << "No MobilityModel" << std::endl;
            }
        }
    }

    /*
    std::vector<double> freqs;
    freqs.reserve(512);
    for (int i = -256; i < 256; ++i)
    {
        freqs.push_back(i*78125 + 5210 * 1e6);
    }
    Ptr<SpectrumModel> rx_spec_model = Create<SpectrumModel>(freqs);

    SpectrumAnalyzerHelper spectrumAnalyzerHelper;
    spectrumAnalyzerHelper.SetChannel(spectrumChannel);

    spectrumAnalyzerHelper.SetRxSpectrumModel(rx_spec_model);
    spectrumAnalyzerHelper.SetPhyAttribute("Resolution", TimeValue(MicroSeconds(100)));
    spectrumAnalyzerHelper.SetPhyAttribute("NoisePowerSpectralDensity",
                                           DoubleValue(1e-20)); // -120 dBm/Hz
    spectrumAnalyzerHelper.EnableAsciiAll("spectrum-analyzer-output");
    NetDeviceContainer spectrumAnalyzerDevices =
        spectrumAnalyzerHelper.Install(wifiApNode);
    */

    // Simulation
    Simulator::Stop(Seconds(2));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    sionnaHelper.Destroy();

    return 0;
}

