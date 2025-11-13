/*
 * Copyright (c) 2025 Yannik Pilz, A. Zubow
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
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>, A. Zubow <zubow@tkn.tu-berlin.de>
 */

#ifndef SIONNA_HELPER_H
#define SIONNA_HELPER_H

#include "message.pb.h"
#include "sionna-utils.h"
#include <zmq.hpp>

namespace ns3
{

/**
 * This helper is used to configure the ns3sionna framework.
 */
class SionnaHelper
{
public:
    /**
     * Selects the Sionna scene and URL to server.
     * @param environment the relative path to the XML file describing the Sionna scene, e.g. "simple_room/simple_room.xml"
     * @param zmq_url the URL of the Python Sionna server component (local or remote)
     */
    SionnaHelper(std::string environment, std::string zmq_url);
    virtual ~SionnaHelper();

    void RandomVariableStreamMessage(ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream* stream,
                                   Ptr<RandomVariableStream> variable);

    /**
     * Configures the radio parameters of the sionna simulation.
     * @param frequency the center frequency [MHz]
     * @param channel_bw the channel bandwidth (including guard bands) [MHz]
     * @param fft_size the FFT used over the channel_bw
     * @param ofdm_subcarrier_spacing the OFDM subcarrier spacing [Hz]
     * @param min_coherence_time_ms the minimum channel coherence time  []in
     * msused for caching
     */
    void Configure(int frequency, int channel_bw, int fft_size, int ofdm_subcarrier_spacing, int min_coherence_time_ms); // both in MHz, ms

    /**
     * Configures the radio parameters of the sionna simulation.
     * @param frequency the center frequency [MHz]
     * @param channel_bw the channel bandwidth (including guard bands) [MHz]
     * @param fft_size the FFT used over the channel_bw
     * @param ofdm_subcarrier_spacing the OFDM subcarrier spacing [Hz]
     */
    void Configure(int frequency, int channel_bw, int fft_size, int ofdm_subcarrier_spacing);

    /**
     * Connect ns3 to Sionna.
     */
    void Start();

    /**
     * Teardown and cleanup
     */
    void Destroy();

    /**
     * Set the mode
     * @param mode MODE_P2P, MODE_P2MP or MODE_P2MP_LAH
     */
    void SetMode(int mode);

    /**
     * Set the submode
     * @param sub_mode the number of computed look-ahead CSI values
     */
    void SetSubMode(int sub_mode);

    double GetNoiseFloor();
    int GetFrequency();

private:
    void SetFrequency(int frequency); // in MHz
    void SetChannelBandwidth(int channel_bw); // in MHz
    void SetFFTSize(int fft_size);
    void SetSubcarrierSpacing(int subcarrier_spacing);

private:
    std::string m_environment; // relative location of XML scenario fi
    int m_mode; // 1=P2P, 2=P2MP, 3=P2MP=LAH
    int m_sub_mode; // used by mode 3
    zmq::context_t m_zmq_context;
    int m_frequency; // in MHz
    int m_channel_bw; // in Mhz
    int m_min_coherence_time_ms; // ms
    int m_fft_size;
    int m_subcarrier_spacing; // in Hz
    double m_noiseDbm;

public:
    zmq::socket_t m_zmq_socket; // ZMQ socket used for connecting ns3 with Sionna

    // possible modes of operation
    const static int MODE_P2P = 1; // only CSI for a single P2P is computed within a single Sionna call
    const static int MODE_P2MP = 2; // a full CSI P2MP (TX to all other RX nodes) is computed within a single Sionna call
    const static int MODE_P2MP_LAH = 3; // same as mode 2 but in addition also future not yet needed channels are computed

    // default value used by ns spectrummodel
    const static int GUARD_MULTIPLIER = 3;
};
} // namespace ns3

#endif // SIONNA_HELPER_H