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

#ifndef SIONNA_PHASED_ARRAY_SPECTRUM_PROPAGATION_LOSS_H
#define SIONNA_PHASED_ARRAY_SPECTRUM_PROPAGATION_LOSS_H

#include "ns3/phased-array-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "sionna-propagation-cache.h"

#include "ns3/channel-condition-model.h"

#include <map>

namespace ns3
{

/**
 * Sionna Phased Array Spectrum Propagation Loss Model
 *
 * TODO: not yet implemented
 */
class SionnaPhasedArraySpectrumPropagationLossModel : public PhasedArraySpectrumPropagationLossModel
{
  public:

    /**
     * Constructor
     */
    SionnaPhasedArraySpectrumPropagationLossModel();

    /**
     * Destructor
     */
    ~SionnaPhasedArraySpectrumPropagationLossModel() override;

    void DoDispose() override;

    /**
     * Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    void SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache);

    /**
     * \brief Assign a fixed random variable stream number to the random variables
     * used by this model.
     *
     * \param stream first stream index to use
     * \return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream);

    /**
     * \brief Compute the received PSD.
     *
     * This function computes the received PSD by applying the Fluctuating Two-Ray (FTR)
     * fast fading model and the beamforming gain.
     * In particular, the beamforming gain is computed as the combination of the single-element
     * and the array gains.
     * The path loss and shadowing are to be computed separately, using the
     * ThreeGppPropagationLossModel class.
     *
     * \param txPsd the PSD of the transmitted signal
     * \param a first node mobility model
     * \param b second node mobility model
     * \param aPhasedArrayModel the antenna array of the first node
     * \param bPhasedArrayModel the antenna array of the second node
     * \return the PSD of the received signal
     */
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(
        Ptr<const SpectrumSignalParameters> txPsd,
        Ptr<const MobilityModel> a,
        Ptr<const MobilityModel> b,
        Ptr<const PhasedArrayModel> aPhasedArrayModel,
        Ptr<const PhasedArrayModel> bPhasedArrayModel) const override;

  private:
    /**
     * Compute the beamforming gain by combining single-element and array gains.
     *
     * Computes the overall beamforming and array gain, assuming analog beamforming
     * both at the transmitter and at the receiver and arbitrary single-element
     * radiation patterns. The computation is performed following Rebato, Mattia, et al.
     * "Study of realistic antenna patterns in 5G mmwave cellular scenarios.",
     * 2018 IEEE International Conference on Communications (ICC). IEEE, 2018.
     *
     * Additionally, whenever the link is in NLOS a penalty factor is introduced, to take into
     * account for the possible misalignment of the beamforming vectors due to the lack of a
     * dominant multipath component. See Kulkarni, Mandar N., Eugene Visotsky, and Jeffrey G.
     * Andrews. "Correction factor for analysis of MIMO wireless networks with highly directional
     * beamforming." IEEE Wireless Communications Letters 7.5 (2018) for further details on this
     * approach.
     *
     * \param a first node mobility model
     * \param b second node mobility model
     * \param aPhasedArrayModel the antenna array of the first node
     * \param bPhasedArrayModel the antenna array of the second node
     * \return the beamforming gain
     */
    double CalcBeamformingGain(Ptr<const MobilityModel> a,
                               Ptr<const MobilityModel> b,
                               Ptr<const PhasedArrayModel> aPhasedArrayModel,
                               Ptr<const PhasedArrayModel> bPhasedArrayModel) const;

    Ptr<SionnaPropagationCache> m_propagationCache;
};

} // namespace ns3

#endif /* SIONNA_PHASED_ARRAY_SPECTRUM_PROPAGATION_LOSS_H */
