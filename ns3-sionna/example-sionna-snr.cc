/*
* Copyright (c) 2025 Zubow
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: zubow@tkn.tu-berlin.de
 */


/**
 * This example shows how to configure the Sionna channel model classes to
 * compute the SNR between two nodes.
 * The simulation involves two static nodes which are placed at a certain
 * distance from each other and communicates through a wireless channel at
 * 2 GHz with a bandwidth of 18 MHz. The default propagation environment is
 * indoor apartment and it can be configured changing the value of the
 * string "scenario".
 * Each node hosts a SimpleNetDevice and has an antenna array with 4 elements.
 */

#include "ns3/channel-condition-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/lte-spectrum-value-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/simple-net-device.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/wifi-spectrum-value-helper.h"
#include "ns3/mobility-module.h"

// Sionna models
#include "lib/sionna-helper.h"
#include "lib/sionna-mobility-model.h"
#include "lib/sionna-phased-array-spectrum-propagation-loss-model.h"
#include "lib/sionna-propagation-cache.h"
#include "lib/sionna-propagation-delay-model.h"
#include "lib/sionna-propagation-loss-model.h"
#include "lib/sionna-utils.h"
#include <fstream>

NS_LOG_COMPONENT_DEFINE("ExampleSionnaSnr");

using namespace ns3;

static Ptr<SionnaPropagationLossModel>
    m_propagationLossModel; //!< the PropagationLossModel object

// Azu: update
static Ptr<SionnaPhasedArraySpectrumPropagationLossModel>
    m_spectrumLossModel; //!< the SpectrumPropagationLossModel object

/**
 * \brief A structure that holds the parameters for the
 * ComputeSnr function. In this way the problem with the limited
 * number of parameters of method Schedule is avoided.
 */
struct ComputeSnrParams
{
    uint16_t freq_0;
    uint16_t channelWidth;
    uint32_t carrierSpacing;
    Ptr<MobilityModel> txMob;        //!< the tx mobility model
    Ptr<MobilityModel> rxMob;        //!< the rx mobility model
    double txPow;                    //!< the tx power in dBm
    double noiseFigure;              //!< the noise figure in dB
    Ptr<PhasedArrayModel> txAntenna; //!< the tx antenna array
    Ptr<PhasedArrayModel> rxAntenna; //!< the rx antenna array
};

/**
 * Perform the beamforming using the DFT beamforming method
 * \param thisDevice the device performing the beamforming
 * \param thisAntenna the antenna object associated to thisDevice
 * \param otherDevice the device towards which point the beam
 */
static void
DoBeamforming(Ptr<NetDevice> thisDevice,
              Ptr<PhasedArrayModel> thisAntenna,
              Ptr<NetDevice> otherDevice)
{
    // retrieve the position of the two devices
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

    // compute the azimuth and the elevation angles
    Angles completeAngle(bPos, aPos);
    double hAngleRadian = completeAngle.GetAzimuth();

    double vAngleRadian = completeAngle.GetInclination(); // the elevation angle

    // retrieve the number of antenna elements and resize the vector
    uint64_t totNoArrayElements = thisAntenna->GetNumberOfElements();
    PhasedArrayModel::ComplexVector antennaWeights(totNoArrayElements);

    // the total power is divided equally among the antenna elements
    double power = 1.0 / sqrt(totNoArrayElements);

    // compute the antenna weights
    const double sinVAngleRadian = sin(vAngleRadian);
    const double cosVAngleRadian = cos(vAngleRadian);
    const double sinHAngleRadian = sin(hAngleRadian);
    const double cosHAngleRadian = cos(hAngleRadian);

    for (uint64_t ind = 0; ind < totNoArrayElements; ind++)
    {
        Vector loc = thisAntenna->GetElementLocation(ind);
        double phase = -2 * M_PI *
                       (sinVAngleRadian * cosHAngleRadian * loc.x +
                        sinVAngleRadian * sinHAngleRadian * loc.y + cosVAngleRadian * loc.z);
        antennaWeights[ind] = exp(std::complex<double>(0, phase)) * power;
    }

    // store the antenna weights
    thisAntenna->SetBeamformingVector(antennaWeights);
}

/**
 * Compute the average SNR
 * \param params A structure that holds the parameters that are needed to perform calculations in
 * ComputeSnr
 */
static void
ComputeSnr(const ComputeSnrParams& params)
{
    // create WiFi 802.11ac in 5GHz

    uint16_t channelWidth = params.channelWidth;
    uint16_t freq = params.freq_0 + (channelWidth / 2); // so as to have 5180/5190/5210/5250 for 20/40/80/160
    uint32_t carrierSpacing = params.carrierSpacing;
    uint16_t guardBandwidth = channelWidth / 5.0; //channelWidth;

    Ptr<SpectrumValue> txPsd =
        WifiSpectrumValueHelper::CreateHeOfdmTxPowerSpectralDensity(freq,
                                                                    channelWidth,
                                                                    1,
                                                                    guardBandwidth,
                                                                    -20.0,
                                                                    -28.0,
                                                                    -40.0); // dBr

    dumpSpectrumValueToFile(txPsd, "he80_txpsd.txt");

    Ptr<SpectrumSignalParameters> txParams = Create<SpectrumSignalParameters>();
    txParams->psd = txPsd->Copy();
    NS_LOG_DEBUG("Average tx power " << 10 * log10(Sum(*txPsd) * carrierSpacing) << " dB");

    Ptr<const SpectrumModel> spectrum_model = txParams->psd->GetSpectrumModel();

    // create the noise PSD
    Ptr<SpectrumValue> noisePsd =
        WifiSpectrumValueHelper::CreateNoisePowerSpectralDensity(freq, channelWidth, carrierSpacing, params.noiseFigure, guardBandwidth);
    NS_LOG_DEBUG("Average noise power " << 10 * log10(Sum(*noisePsd) * carrierSpacing) << " dB");

    // apply the pathloss
    double propagationGainDb = m_propagationLossModel->CalcRxPower(0, params.txMob, params.rxMob);
    NS_LOG_DEBUG("Pathloss " << -propagationGainDb << " dB");
    double propagationGainLinear = std::pow(10.0, (propagationGainDb) / 10.0);
    *(txParams->psd) *= propagationGainLinear;

    NS_ASSERT_MSG(params.txAntenna, "params.txAntenna is nullptr!");
    NS_ASSERT_MSG(params.rxAntenna, "params.rxAntenna is nullptr!");

    // apply the fast fading and the beamforming gain
    Ptr<SpectrumValue> rxPsd = m_spectrumLossModel->CalcRxPowerSpectralDensity(txParams,
                                                                               params.txMob,
                                                                               params.rxMob,
                                                                               params.txAntenna,
                                                                               params.rxAntenna);
    NS_LOG_DEBUG("Average rx power " << 10 * log10(Sum(*rxPsd) * carrierSpacing) << " dB");

    // compute the SNR
    NS_LOG_DEBUG("Average SNR " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " dB");

    // print the SNR and pathloss values in the snr-trace.txt file
    std::ofstream f;
    f.open("snr-trace.txt", std::ios::out | std::ios::app);
    f << Simulator::Now().GetSeconds() << " " << 10 * log10(Sum(*rxPsd) / Sum(*noisePsd)) << " "
      << propagationGainDb << std::endl;
    f.close();
}

int
main(int argc, char* argv[])
{
    bool verbose = true;
    bool caching = true;
    double distance = 2.0;
    uint16_t freq_0 = 5170;       // in MHz
    uint16_t channelWidth = 80; //40;  // in MHz
    uint32_t carrierSpacing = 312500;      // in Hz
    double txPow = 20.0;          // tx power in dBm
    double noiseFigure = 9.0;     // noise figure in dB
    uint32_t simTime = 1000;      // simulation time in milliseconds
    uint32_t timeRes = 10;        // time resolution in milliseconds
    std::string environment = "2_rooms_with_door/2_rooms_with_door_open.xml";

    if (verbose)
    {
        LogComponentEnable("ExampleSionnaSnr", LOG_DEBUG);
        LogComponentEnable("SionnaPropagationLossModel", LOG_INFO);
        LogComponentEnable("SionnaPropagationCache", LOG_INFO);
    }

    //Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
    //                   TimeValue(MilliSeconds(1))); // update the channel at each iteration
    //Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
    //                   TimeValue(MilliSeconds(0.0))); // do not update the channel condition

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    std::cout << "Example spectrum scenario with sionna" << std::endl << std::endl;
    SionnaHelper sionnaHelper(environment, "tcp://localhost:5555");

    Ptr<SionnaPropagationCache> propagationCache = CreateObject<SionnaPropagationCache>();
    propagationCache->SetSionnaHelper(sionnaHelper);
    propagationCache->SetCaching(caching);

    // create and configure the factories for the channel condition and propagation loss models
    //ObjectFactory propagationLossModelFactory;
    ObjectFactory channelConditionModelFactory;

    //propagationLossModelFactory.SetTypeId(ThreeGppUmaPropagationLossModel::GetTypeId());
    channelConditionModelFactory.SetTypeId(ThreeGppUmaChannelConditionModel::GetTypeId());

    // create the propagation loss model
    m_propagationLossModel = CreateObject<SionnaPropagationLossModel>();
    m_propagationLossModel->SetPropagationCache(propagationCache);
    //m_propagationLossModel->SetAttribute("Frequency", DoubleValue(frequency));
    //m_propagationLossModel->SetAttribute("ShadowingEnabled", BooleanValue(false));


    // create the spectrum propagation loss model
    m_spectrumLossModel = CreateObject<SionnaPhasedArraySpectrumPropagationLossModel>();
    m_spectrumLossModel->SetPropagationCache(propagationCache);
    //m_spectrumLossModel->SetChannelModelAttribute("Frequency", DoubleValue(freq_0 * 1e6));
    //std::string scenario = "UMa"; // 3GPP propagation scenario
    //m_spectrumLossModel->SetChannelModelAttribute("Scenario", StringValue(scenario));

    // create the channel condition model and associate it with the spectrum and
    // propagation loss model
    //Ptr<ChannelConditionModel> condModel =
    //    channelConditionModelFactory.Create<ThreeGppChannelConditionModel>();
    //m_spectrumLossModel->SetChannelModelAttribute("ChannelConditionModel", PointerValue(condModel));
    //m_propagationLossModel->SetChannelConditionModel(condModel);

    //////////////////
    // create the tx and rx nodes
    NodeContainer nodes;
    nodes.Create(2);

    // create the tx and rx devices
    Ptr<SimpleNetDevice> txDev = CreateObject<SimpleNetDevice>();
    Ptr<SimpleNetDevice> rxDev = CreateObject<SimpleNetDevice>();

    // associate the nodes and the devices
    nodes.Get(0)->AddDevice(txDev);
    txDev->SetNode(nodes.Get(0));
    nodes.Get(1)->AddDevice(rxDev);
    rxDev->SetNode(nodes.Get(1));

    // Mobility configuration
    MobilityHelper mobility;

    mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    mobility.Install(nodes);

    // create the tx and rx mobility models, set the positions
    Ptr<MobilityModel> txMob = nodes.Get(0)->GetObject<MobilityModel>();
    txMob->SetPosition(Vector(5.0, 2.05, 1.0));
    Ptr<MobilityModel> rxMob = nodes.Get(1)->GetObject<MobilityModel>();
    rxMob->SetPosition(Vector(5.0 + distance, 2.0, 1.0));

    // assign the mobility models to the nodes
    //nodes.Get(0)->AggregateObject(txMob);
    //nodes.Get(1)->AggregateObject(rxMob);

    // create the antenna objects and set their dimensions
    Ptr<PhasedArrayModel> txAntenna =
        CreateObjectWithAttributes<UniformPlanarArray>("NumColumns",
                                                       UintegerValue(2),
                                                       "NumRows",
                                                       UintegerValue(2));
    Ptr<PhasedArrayModel> rxAntenna =
        CreateObjectWithAttributes<UniformPlanarArray>("NumColumns",
                                                       UintegerValue(2),
                                                       "NumRows",
                                                       UintegerValue(2));

    // set the beamforming vectors
    DoBeamforming(txDev, txAntenna, rxDev);
    DoBeamforming(rxDev, rxAntenna, txDev);

    for (int i = 0; i < floor(simTime / timeRes); i++)
    {
        ComputeSnrParams params{freq_0, channelWidth, carrierSpacing, txMob, rxMob, txPow, noiseFigure, txAntenna, rxAntenna};
        Simulator::Schedule(MilliSeconds(timeRes * i), &ComputeSnr, params);
    }

    // set center frequency & bandwidth for Sionna
    WifiStandard wifi_standard = WIFI_STANDARD_80211ax; // WIFI6
    sionnaHelper.Configure(freq_0, channelWidth, getFFTSize(wifi_standard, channelWidth), getSubcarrierSpacing(wifi_standard));

    sionnaHelper.Start();

    Simulator::Run();
    Simulator::Destroy();

    sionnaHelper.Destroy();

    return 0;
}
