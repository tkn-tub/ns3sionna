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

#ifndef SIONNA_UTILS_TAG_H
#define SIONNA_UTILS_TAG_H

#include "ns3/simple-ref-count.h"
#include <vector>      // For std::vector
#include <complex>     // For std::complex<double>
#include "ns3/spectrum-value.h"
#include "ns3/wifi-standards.h"
#include <fstream>
#include <ns3/log.h>
#include <ns3/wifi-phy.h>
#include <ns3/node-container.h>
#include <ns3/wifi-net-device.h>
#include <ns3/yans-wifi-phy.h>
#include "sionna-mobility-model.h"

/**
 * Collection of useful functions
 */
namespace ns3 {

    inline uint32_t ContextToNodeId(std::string context)
    {
        std::string sub = context.substr(10);
        uint32_t pos = sub.find("/Device");
        return std::stoi(sub.substr(0, pos));
    }

    inline void dumpSpectrumValueToFile(Ptr<SpectrumValue> psd, const std::string& filename) {

        std::ofstream outFile(filename);

        if (!outFile.is_open()) {
            std::cerr << "Failed to open file " << filename << std::endl;
            return;
        }

        // Dump all PSD values (one per line)
        for (size_t i = 0; i < psd->GetValuesN(); ++i) {
            outFile << (*psd)[i] << "\n";  // psd is like a vector
        }

        outFile.close();
        std::cout << "PSD values dumped to " << filename << std::endl;
    }

    inline void dumpComplexVecToStream(std::vector<std::complex<double>> H, std::ofstream& ofs) {
        // Dump all values (one per line)
        for (size_t i = 0; i < H.size(); i++) {
            if (H[i].imag() >= 0.0)
            {
                ofs << H[i].real() << "+" << H[i].imag() << "j" << "\n";
            } else
            {
                ofs << H[i].real() << "" << H[i].imag() << "j" << "\n";
            }
        }
    }

    inline void dumpComplexVecToFile(std::vector<std::complex<double>> H, const std::string& filename)
    {
        std::ofstream outFile(filename);

        if (!outFile.is_open()) {
            std::cerr << "Failed to open file " << filename << std::endl;
            return;
        }

        dumpComplexVecToStream(H, outFile);

        outFile.close();
    }

    inline int getFFTSize(WifiStandard wifi_standard, double channel_bw)
    {
        switch (wifi_standard)
        {
            case WIFI_STANDARD_UNSPECIFIED:
            case WIFI_STANDARD_80211b:
            case WIFI_STANDARD_80211ad:
                NS_ASSERT(false); // not yet supported
                return -1;

            case WIFI_STANDARD_80211a:
            case WIFI_STANDARD_80211g:
            case WIFI_STANDARD_80211p:
                return 64;

            case WIFI_STANDARD_80211n:
            case WIFI_STANDARD_80211ac:
                return 64 * (channel_bw/20);

            case WIFI_STANDARD_80211ax:
            case WIFI_STANDARD_80211be:
                return 256 * (channel_bw/20);

            default:
                NS_ASSERT(false);
                return -1;
        }
    }

inline int getSubcarrierSpacing(WifiStandard wifi_standard)
    {
        switch (wifi_standard)
        {
        case WIFI_STANDARD_UNSPECIFIED:
        case WIFI_STANDARD_80211b:
        case WIFI_STANDARD_80211ad:
            NS_ASSERT(false); // not yet supported
            return -1;

        case WIFI_STANDARD_80211a:
        case WIFI_STANDARD_80211g:
        case WIFI_STANDARD_80211p:
        case WIFI_STANDARD_80211n:
        case WIFI_STANDARD_80211ac:
            return 312500;

        case WIFI_STANDARD_80211ax:
        case WIFI_STANDARD_80211be:
            return 78125;

        default:
            NS_ASSERT(false);
            return -1;
        }
    }

    inline double get_center_freq(Ptr<NetDevice> nd)
    {
        Ptr<WifiPhy> wp = nd->GetObject<WifiNetDevice>()->GetPhy();
        return wp->GetObject<WifiPhy>()->GetFrequency();
    }

    inline double get_channel_width(Ptr<NetDevice> nd)
    {
        Ptr<WifiPhy> wp = nd->GetObject<WifiNetDevice>()->GetPhy();
        return wp->GetObject<WifiPhy>()->GetChannelWidth();
    }


} // namespace ns3

#endif /* SIONNA_UTILS_TAG_H */