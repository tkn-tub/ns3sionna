/*
* Copyright (c) 2025
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: A. Zubow <zubow@tkn.tu-berlin.de>
 */


#include "sionna-phased-array-spectrum-propagation-loss-model.h"

#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/object-factory.h>
#include <ns3/pointer.h>
#include <ns3/random-variable-stream.h>
#include <ns3/string.h>

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPhasedArraySpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaPhasedArraySpectrumPropagationLossModel);

SionnaPhasedArraySpectrumPropagationLossModel::SionnaPhasedArraySpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(false, "Not yet implemented");
}

SionnaPhasedArraySpectrumPropagationLossModel::~SionnaPhasedArraySpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

void
SionnaPhasedArraySpectrumPropagationLossModel::DoDispose()
{
}

void
SionnaPhasedArraySpectrumPropagationLossModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}

TypeId
SionnaPhasedArraySpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPhasedArraySpectrumPropagationLossModel")
            .SetParent<PhasedArraySpectrumPropagationLossModel>()
            .SetGroupName("Sionna")
            .AddConstructor<SionnaPhasedArraySpectrumPropagationLossModel>();
            //.AddAttribute(
            //    "ChannelConditionModel",
            //    "Pointer to the channel condition model.",
            //    PointerValue(),
            //    MakePointerAccessor(&TwoRaySpectrumPropagationLossModel::m_channelConditionModel),
            //    MakePointerChecker<ChannelConditionModel>())
            //.AddAttribute(
            //    "Scenario",
            //    "The 3GPP scenario (RMa, UMa, UMi-StreetCanyon, InH-OfficeOpen, InH-OfficeMixed).",
            //    StringValue("RMa"),
            //    MakeStringAccessor(&TwoRaySpectrumPropagationLossModel::SetScenario),
            //    MakeStringChecker())
            //.AddAttribute("Frequency",
            //              "The operating Frequency in Hz",
            //              DoubleValue(500.0e6),
            //              MakeDoubleAccessor(&TwoRaySpectrumPropagationLossModel::SetFrequency),
            //              MakeDoubleChecker<double>());
    return tid;
}


double
SionnaPhasedArraySpectrumPropagationLossModel::CalcBeamformingGain(
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);

    // Get the relative angles between tx and rx phased arrays
    Angles aAngle(b->GetPosition(), a->GetPosition());
    Angles bAngle(a->GetPosition(), b->GetPosition());

    // Compute the beamforming vectors and and array responses
    auto aArrayResponse = aPhasedArrayModel->GetSteeringVector(aAngle);
    auto aAntennaFields = aPhasedArrayModel->GetElementFieldPattern(aAngle);
    auto aBfVector = aPhasedArrayModel->GetBeamformingVector();
    auto bArrayResponse = bPhasedArrayModel->GetSteeringVector(bAngle);
    auto bAntennaFields = bPhasedArrayModel->GetElementFieldPattern(bAngle);
    auto bBfVector = bPhasedArrayModel->GetBeamformingVector();

    std::complex<double> aArrayOverallResponse = 0;
    std::complex<double> bArrayOverallResponse = 0;

    // Compute the dot products between the array responses and the beamforming vectors
    for (size_t i = 0; i < aPhasedArrayModel->GetNumberOfElements(); i++)
    {
        aArrayOverallResponse += aArrayResponse[i] * aBfVector[i];
    }
    for (size_t i = 0; i < bPhasedArrayModel->GetNumberOfElements(); i++)
    {
        bArrayOverallResponse += bArrayResponse[i] * bBfVector[i];
    }

    double gain = norm(aArrayOverallResponse) *
                  (std::pow(aAntennaFields.first, 2) + std::pow(aAntennaFields.second, 2)) *
                  norm(bArrayOverallResponse) *
                  (std::pow(bAntennaFields.first, 2) + std::pow(bAntennaFields.second, 2));

    // Retrieve LOS condition to check if a correction factor needs to be introduced
    //ChannelCondition::LosConditionValue cond = GetLosCondition(a, b);
    //if (cond == ChannelCondition::NLOS)
    //{
        // The linear penalty factor to be multiplied to the beamforming gain whenever the link is
        // in NLOS
    //    constexpr double NLOS_BEAMFORMING_FACTOR = 1.0 / 19;
    //    gain *= NLOS_BEAMFORMING_FACTOR;
    //}

    return gain;
}

Ptr<SpectrumValue>
SionnaPhasedArraySpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b,
    Ptr<const PhasedArrayModel> aPhasedArrayModel,
    Ptr<const PhasedArrayModel> bPhasedArrayModel) const
{
    NS_LOG_FUNCTION(this);
    uint32_t aId = a->GetObject<Node>()->GetId(); // Id of the node a
    uint32_t bId = b->GetObject<Node>()->GetId(); // Id of the node b

    NS_ASSERT_MSG(aId != bId, "The two nodes must be different from one another");
    NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
                  "The position of a and b devices cannot be the same");

    Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(params->psd);

    // Retrieve the antenna of device a
    NS_ASSERT_MSG(aPhasedArrayModel, "Antenna not found for node " << aId);
    NS_LOG_DEBUG("a node " << a->GetObject<Node>() << " antenna " << aPhasedArrayModel);

    // Retrieve the antenna of the device b
    NS_ASSERT_MSG(bPhasedArrayModel, "Antenna not found for device " << bId);
    NS_LOG_DEBUG("b node " << bId << " antenna " << bPhasedArrayModel);

    // Retrieve FTR params from table
    //FtrParams ftrParams = GetFtrParameters(a, b);

    // get small-scale fading matrix
    std::vector<std::complex<double>> H_norm = m_propagationCache->GetPropagationCSI(a, b);
    // todo: apply

    auto vit = rxPsd->ValuesBegin();      // psd iterator
    //auto sbit = tempPsd->ConstBandsBegin(); // band iterator
    int cnt = 0;
    while (vit != rxPsd->ValuesEnd())
    {
        cnt++;
        vit++;
    }
    std::cout << cnt << std::endl;

    // Compute the beamforming gain
    double bfGain = 1.0; //CalcBeamformingGain(a, b, aPhasedArrayModel, bPhasedArrayModel);

    // Apply the above terms to the TX PSD
    *rxPsd *= (1.0 * bfGain);

    return rxPsd;
}


int64_t
SionnaPhasedArraySpectrumPropagationLossModel::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    //m_normalRv->SetStream(stream);
    //m_uniformRv->SetStream(stream + 1);
    //m_gammaRv->SetStream(stream + 2);
    return 0; //3;
}

} // namespace ns3
