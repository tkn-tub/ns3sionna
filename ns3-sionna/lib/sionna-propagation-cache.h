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

#ifndef SIONNA_PROPAGATION_CACHE_H
#define SIONNA_PROPAGATION_CACHE_H
 
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include <complex>
#include "sionna-helper.h"

#include <map>

#include <ns3/propagation-delay-model.h>
#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * All CSI values are cached within ns3sionna framework for faster simulation time.
 *
 * TODO: garbage collection - purge too old values from cache
 */
class SionnaPropagationCache : public ns3::Object
{
    public:
        static TypeId GetTypeId();

        SionnaPropagationCache();
        ~SionnaPropagationCache();

        // propagation delay between two nodes
        Time GetPropagationDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;
        // average propagation loss
        double GetPropagationLoss(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        double GetPropagationLoss(Ptr<MobilityModel> a, Ptr<MobilityModel> b, double txPowerDbm) const;
        // small-scale fading
        std::vector<std::complex<double>> GetPropagationCSI(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
        // frequency of subcarriers
        std::vector<int> GetPropagationFreq(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;

        void SetSionnaHelper(SionnaHelper &sionnaHelper);
        SionnaHelper* GetSionnaHelper();
        void SetCaching(bool caching);
        void SetOptimize(bool optimize);
        double GetStats();
        void PrintStats();

    private:
        struct CacheKey
        {
        /**
         * @brief Constructs a new CacheKey
         * @param a TX node ID
         * @param b RX node ID
         */
        CacheKey(uint32_t a, uint32_t b)
                : m_first(a < b ? a : b),
                  m_second(a < b ? b : a)
            {
            }

            uint32_t m_first;
            uint32_t m_second;

            bool operator<(const CacheKey& other) const
            {
                if (m_first != other.m_first)
                {
                    return m_first < other.m_first;
                }
                else
                {
                    return m_second < other.m_second;
                }
            }
        };

        struct CacheEntry
        {
            /**
            * @brief Constructs a new CacheEntry
            * @param delay the estimated propagation delay between TX and RX node.
            * @param loss the estimated wideband pathloss
            * @param start_time the point in time when the CSI was estimated
            * @param end_time valid until (=start_time + computed channel coherence time)
            * @param num_ofdm_subcarrier the number of OFDM subcarriers in CFR
            */
            CacheEntry(Time delay, double loss, Time start_time, Time end_time, int num_ofdm_subcarrier,
                uint32_t a, uint32_t b, Vector a_position, Vector b_position)
                : m_delay(delay),
                  m_loss(loss),
                  m_start_time(start_time),
                  m_end_time(end_time),
                  m_num_ofdm_subcarrier(num_ofdm_subcarrier),
                  m_a(a),
                  m_b(b),
                  m_a_position(a_position),
                  m_b_position(b_position)
            {
                m_cfr.reserve(num_ofdm_subcarrier);
            }

            CacheEntry(): m_start_time(-1), m_end_time(-1)
            {
            }

            Time m_delay;
            double m_loss;
            Time m_start_time;
            Time m_end_time;
            int m_num_ofdm_subcarrier;
            uint32_t m_a;
            uint32_t m_b;
            Vector m_a_position;
            Vector m_b_position;
            // optional
            std::vector<int> m_freq;
            std::vector<std::complex<double>> m_cfr; // channel frequency response
        };

        CacheEntry GetPropagationData(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

        SionnaHelper *m_sionnaHelper;
        bool m_caching;
        typedef std::map<CacheKey, std::vector<CacheEntry>> Cache;
        mutable Cache m_cache;
        mutable double m_cache_hits;
        mutable double m_cache_miss;
        bool m_optimize; // too far distance are not computed with raytracing
        const double m_optimize_margin = 0;
        Ptr<FriisPropagationLossModel> m_friisLossModel;
        Ptr<ConstantSpeedPropagationDelayModel> m_constSpeedDelayModel;
};

} // namespace ns3
 
#endif // SIONNA_PROPAGATION_CACHE_H
