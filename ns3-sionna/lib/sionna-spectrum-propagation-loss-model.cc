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

#include "sionna-spectrum-propagation-loss-model.h"
#include "sionna-utils.h"

#include "ns3/socket.h"
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/object-factory.h>
#include <ns3/pointer.h>
#include <ns3/random-variable-stream.h>
#include <ns3/spectrum-signal-parameters.h>
#include <ns3/string.h>
#include <ns3/wifi-phy.h>
#include <ns3/wifi-ppdu.h>
#include <ns3/wifi-psdu.h>
#include <ns3/wifi-spectrum-phy-interface.h>
#include <ns3/wifi-spectrum-signal-parameters.h>

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaSpectrumPropagationLossModel);

SionnaSpectrumPropagationLossModel::SionnaSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

SionnaSpectrumPropagationLossModel::~SionnaSpectrumPropagationLossModel()
{
    NS_LOG_FUNCTION(this);
}

TypeId
SionnaSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaSpectrumPropagationLossModel")
            .SetParent<SpectrumPropagationLossModel>()
            .SetGroupName("Sionna")
            .AddConstructor<SionnaSpectrumPropagationLossModel>();
    return tid;
}

void
SionnaSpectrumPropagationLossModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}


Ptr<SpectrumValue>
SionnaSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
{
    NS_LOG_FUNCTION(this);
    uint32_t aId = a->GetObject<Node>()->GetId(); // Id of the node a
    uint32_t bId = b->GetObject<Node>()->GetId(); // Id of the node b

    NS_LOG_DEBUG(std::fixed << std::setprecision(9) << Simulator::Now().GetSeconds() <<"s: DoCalcRxPowerSpectralDensity for link " << aId << " - " << bId);

    NS_ASSERT_MSG(aId != bId, "The two nodes must be different from one another");
    NS_ASSERT_MSG(a->GetDistanceFrom(b) > 0.0,
                  "The position of a and b devices cannot be the same");

    // get PSD size
    auto spectrum_bin_size
        = (params->psd->GetSpectrumModel()->Begin()+1)->fc - (params->psd->GetSpectrumModel()->Begin()->fc);
    auto spectrum_dim
        = (params->psd->GetSpectrumModel()->End()-1)->fc - (params->psd->GetSpectrumModel()->Begin()->fc);

    NS_LOG_DEBUG("\t PSD size: " << params->psd->GetValuesN() << " df=" << spectrum_bin_size
        << "Hz, " << "Total=" << spectrum_dim << "Hz");

    Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue>(params->psd);

    // wideband pathloloss
    double wb_loss = m_propagationCache->GetPropagationLoss(a, b);

    // get small-scale fading matrix
    std::vector<std::complex<double>> H_norm = m_propagationCache->GetPropagationCSI(a, b);
    // add trailing 1 to have same size
    H_norm.emplace_back(1.0, 0.0);

    NS_ASSERT_MSG(H_norm.size() == params->psd->GetValuesN(), "PSD and CFR must have the same size");

    // apply small-scale fading
    auto vit = rxPsd->ValuesBegin(); // psd value iterator
    size_t idx = 0;
    while (vit != rxPsd->ValuesEnd())
    {
        // multiply PSD with |H|^2
        *vit = *vit * std::norm(H_norm[idx]);
        vit++; idx++;
    }

    // tag the packet payload with CFR for later processing in application layer
    if (auto wifiTxParams = DynamicCast<const WifiSpectrumSignalParameters>(params))
    {   // WiFi packet found
        Ptr<WifiPpdu> ppdu = wifiTxParams->ppdu->Copy();
        Ptr<const WifiPsdu> psdu = ppdu->GetPsdu();

        // for each payload
        for (size_t payload_idx = 0; payload_idx < psdu->GetNMpdus(); payload_idx++) {
            Ptr<const Packet> p = psdu->GetPayload(payload_idx);

            CFRTag tag;
            if (p->PeekPacketTag(tag))
            {
                //std::cout << "tag already exists; replace H" << std::endl;
                tag.SetComplexes(H_norm);
                tag.SetPathloss(wb_loss);
            } else
            {
                //std::cout << "add new tag with H" << std::endl;
                tag.SetComplexes(H_norm);
                tag.SetPathloss(wb_loss);
                p->AddPacketTag(tag);
            }
        }
    } else
    {
        // TODO: support of LTE
        NS_LOG_WARN("Unknown SpectrumSignalParameters: " << typeid(params).name() << "; packet not annotated with CSI");
    }

    return rxPsd;
}

} // namespace ns3
