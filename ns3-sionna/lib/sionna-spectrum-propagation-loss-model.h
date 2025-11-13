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

#ifndef SIONNA_SPECTRUM_PROPAGATION_LOSS_H
#define SIONNA_SPECTRUM_PROPAGATION_LOSS_H

#include "ns3/spectrum-propagation-loss-model.h"
#include "sionna-propagation-cache.h"
#include "ns3/random-variable-stream.h"
#include <ns3/nstime.h>
#include <ns3/object.h>
#include "cfr-tag.h"

#include <map>

namespace ns3
{

/**
 * The fast fading loss model computed in Sionna. Note: it is normalized to 1.
 */
class SionnaSpectrumPropagationLossModel : public SpectrumPropagationLossModel
{
  public:
    SionnaSpectrumPropagationLossModel();
    ~SionnaSpectrumPropagationLossModel() override;

    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    void SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache);

  private:
    /**
     * @param params the spectrum signal parameters.
     * \param a sender mobility
     * \param b receiver mobility
     * \return set of values vs frequency representing the received
     *         power in the same units used for the txPsd parameter.
     */
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;

    Ptr<SionnaPropagationCache> m_propagationCache;
};

} // namespace ns3

#endif /* SIONNA_SPECTRUM_PROPAGATION_LOSS_H */
