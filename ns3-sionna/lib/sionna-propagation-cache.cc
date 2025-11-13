/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-propagation-cache.h"

#include "message.pb.h"
#include "sionna-mobility-model.h"

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include <boost/units/physical_dimensions/volume.hpp>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPropagationCache");

NS_OBJECT_ENSURE_REGISTERED(SionnaPropagationCache);

TypeId
SionnaPropagationCache::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPropagationCache")
            .SetParent<Object>()
            .SetGroupName("Propagation")
            .AddConstructor<SionnaPropagationCache>();
    return tid;
}

SionnaPropagationCache::SionnaPropagationCache()
    : m_sionnaHelper(nullptr), m_caching(true), m_cache_hits(0), m_cache_miss(0), m_optimize(true)
{
    m_friisLossModel = CreateObject<FriisPropagationLossModel>();
    m_constSpeedDelayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
}

SionnaPropagationCache::~SionnaPropagationCache()
{
    m_cache.clear();
}

Time
SionnaPropagationCache::GetPropagationDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    // Check if distance is too far so that a simpler model can be used
    if (m_optimize)
    {
        const double MAX_TXPOWER_DBM = 20.0; // AZU: todo: hardcoded
        double resultdBm = m_friisLossModel->CalcRxPower(MAX_TXPOWER_DBM, a, b);
        if (resultdBm + m_optimize_margin < m_sionnaHelper->GetNoiseFloor())
        {
            Time const_delay = m_constSpeedDelayModel->GetDelay(a, b);
            NS_LOG_DEBUG("Skipped raytracing for prop delay due to large distance; const delay used: " << const_delay);
            return const_delay;
        }
        // signal is too strong and delay need to be computed with ray tracing
    }

    return GetPropagationData(a, b).m_delay;
}

double
SionnaPropagationCache::GetPropagationLoss(Ptr<MobilityModel> a, Ptr<MobilityModel> b, double txPowerDbm) const
{
    // Check if distance is too far so that a simpler model can be used
    if (m_optimize)
    {
        double resultdBm = m_friisLossModel->CalcRxPower(txPowerDbm, a, b);
        if (resultdBm + m_optimize_margin < m_sionnaHelper->GetNoiseFloor())
        {
            double friis_loss = (-1) * (resultdBm - txPowerDbm);
            NS_LOG_DEBUG("Skipped raytracing for prop loss due to large distance; friis loss used: " << friis_loss);
            return friis_loss;
        }
        // signal is too strong and need to be computed with ray tracing
    }

    Vector pos_a = a->GetPosition();
    Vector pos_b = b->GetPosition();

    // update position on mobility models to reflect node position in Sionna
    CacheEntry ce = GetPropagationData(a, b);

    Ptr<Node> node_a = a->GetObject<Node>();
    Ptr<Node> node_b = b->GetObject<Node>();

    // needed as the cache assumes channel reciprocity
    if (node_a->GetId() == ce.m_a)
    {
        a->SetPosition(ce.m_a_position);
        b->SetPosition(ce.m_b_position);
    } else
    {
        a->SetPosition(ce.m_b_position);
        b->SetPosition(ce.m_a_position);
    }

    Time current_time = Simulator::Now();
    if (pos_a != a->GetPosition())
    {
        NS_LOG_INFO("ns3sionna::update Pos for node: " << node_a->GetId() << " from " <<
            ": (" << pos_a.x << "," << pos_a.y << "," << pos_a.z << ") to: " <<
            ": (" << a->GetPosition().x << "," << a->GetPosition().y << "," << a->GetPosition().z << ")");
    }

    if (pos_b != b->GetPosition())
    {
        NS_LOG_INFO("ns3sionna::update Pos for node: " << node_b->GetId() << " from " <<
            ": (" << pos_b.x << "," << pos_b.y << "," << pos_b.z << ") to: " <<
            ": (" << b->GetPosition().x << "," << b->GetPosition().y << "," << b->GetPosition().z << ")");
    }

    return ce.m_loss;
}

double
SionnaPropagationCache::GetPropagationLoss(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
{
    Ptr<MobilityModel> tmp_a = ConstCast<MobilityModel>(a);
    Ptr<MobilityModel> tmp_b = ConstCast<MobilityModel>(b);

    return GetPropagationData(tmp_a, tmp_b).m_loss;
}

std::vector<int>
SionnaPropagationCache::GetPropagationFreq(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
{
    Ptr<MobilityModel> tmp_a = ConstCast<MobilityModel>(a);
    Ptr<MobilityModel> tmp_b = ConstCast<MobilityModel>(b);
    return GetPropagationData(tmp_a, tmp_b).m_freq;
}


std::vector<std::complex<double>>
SionnaPropagationCache::GetPropagationCSI(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const
{
    Ptr<MobilityModel> tmp_a = ConstCast<MobilityModel>(a);
    Ptr<MobilityModel> tmp_b = ConstCast<MobilityModel>(b);
    return GetPropagationData(tmp_a, tmp_b).m_cfr;
}

void
SionnaPropagationCache::SetSionnaHelper(SionnaHelper &sionnaHelper)
{
    m_sionnaHelper = &sionnaHelper;
}

SionnaHelper*
SionnaPropagationCache::GetSionnaHelper()
{
    return m_sionnaHelper;
}

void
SionnaPropagationCache::SetCaching(bool caching)
{
    m_caching = caching;
}

void
SionnaPropagationCache::SetOptimize(bool optimize)
{
    m_optimize = optimize;
}

double
SionnaPropagationCache::GetStats()
{
    double ratio = m_cache_hits / (m_cache_hits + m_cache_miss);
    return ratio;
}

void SionnaPropagationCache::PrintStats()
{
    std::cout << "Ns3-sionna: cache #lookups: " <<  (m_cache_hits + m_cache_miss) << ", #misses:"
        << m_cache_miss << ", hit ratio: " <<  this->GetStats() << std::endl;
}


SionnaPropagationCache::CacheEntry
SionnaPropagationCache::GetPropagationData(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    NS_ASSERT_MSG(m_sionnaHelper, "SionnaPropagationCache must have reference to SionnaHelper.");
    Ptr<SionnaMobilityModel> sionna_a = DynamicCast<SionnaMobilityModel> (a);
    Ptr<SionnaMobilityModel> sionna_b = DynamicCast<SionnaMobilityModel> (b);
    NS_ASSERT_MSG(sionna_a && sionna_b, "Not using SionnaMobilityModel.");

    Time current_time = Simulator::Now();

    Ptr<Node> node_a = a->GetObject<Node>();
    Ptr<Node> node_b = b->GetObject<Node>();
    NS_ASSERT_MSG(node_a && node_b, "Nodes not found.");

    NS_LOG_DEBUG("ns3sionna::GetPropagationData for lnk: " << node_a->GetId() << " to " << node_b->GetId());

    if (m_caching)
    {
        //NS_LOG_INFO("Check cache");
        // Look in the cache and check if delay and loss have already been calculated for the two nodes
        CacheKey key = CacheKey(node_a->GetId(), node_b->GetId());
        auto it = m_cache.find(key);

        if (it != m_cache.end())
        {
            //NS_LOG_INFO(" Cache entry found but need to check if expired ");
            // remove outdated entries
            std::vector<CacheEntry>::iterator vec_it = it->second.begin();
            while (vec_it != it->second.end()) {
                if (vec_it->m_end_time < current_time) {
                    vec_it = it->second.erase(vec_it);
                } else {
                    ++vec_it;
                }
            }

            // iterate over all stored entries for that link
            for (CacheEntry c_entry : it->second) {
                //NS_LOG_INFO(" " << c_entry.m_start_time << " " << c_entry.m_end_time << " " << current_time);
                // If delay and loss exist in the cache, check if the entry is not outdated
                if (c_entry.m_end_time >= current_time && c_entry.m_start_time <= current_time)
                {
                    NS_LOG_DEBUG("\t: Cache hit for lnk: " << node_a->GetId() << " to " << node_b->GetId());
                    m_cache_hits += 1;
                    // Return cache entry as the value is still fresh
                    return c_entry;
                }
            }
        }
    }

    NS_LOG_INFO("\t: Cache miss for lnk: " << node_a->GetId() << " to " << node_b->GetId());
    m_cache_miss += 1;

    // Prepare the request message
    ns3sionna::Wrapper wrapper;

    // Fill the information message
    ns3sionna::ChannelStateRequest* propagation_request = wrapper.mutable_channel_state_request();
    propagation_request->set_tx_node(node_a->GetId());
    propagation_request->set_rx_node(node_b->GetId());
    propagation_request->set_time(current_time.GetNanoSeconds());
    
    // Serialize the request message
    std::string serialized_message;
    wrapper.SerializeToString(&serialized_message);

    // Send the request message
    zmq::message_t zmq_message(serialized_message.data(), serialized_message.size());
    m_sionnaHelper->m_zmq_socket.send(zmq_message, zmq::send_flags::none);

    // Receive the reply message
    zmq::message_t zmq_reply;
    zmq::recv_result_t result = m_sionnaHelper->m_zmq_socket.recv(zmq_reply, zmq::recv_flags::none);
    
    NS_ASSERT_MSG(result, "Failed to receive reply after propagation request message.");

    // Check if the reply message is a propagation response
    ns3sionna::Wrapper reply_wrapper;
    reply_wrapper.ParseFromArray(zmq_reply.data(), zmq_reply.size());
    //NS_LOG_INFO("ZMQ::CSI_RESP sz=" << zmq_reply.size() << " Bytes");

    NS_ASSERT_MSG(reply_wrapper.has_channel_state_response(), "Reply after channel state request is not a channel state response.");

    // Extract the delay, loss and time to live value
    const ns3sionna::ChannelStateResponse& csi_response = reply_wrapper.channel_state_response();

    NS_LOG_INFO("ns3sionna::Req CSI data from sionna: #samples=" << csi_response.csi_size());

    // result contains also future CSI; fill-up the cache
    for (int csi_i=0; csi_i < csi_response.csi_size(); csi_i++) {
        Time start_time = NanoSeconds(csi_response.csi(csi_i).start_time());
        Time end_time = NanoSeconds(csi_response.csi(csi_i).end_time());

        NS_LOG_DEBUG("\t\t: CSI ts= (" << start_time.GetNanoSeconds() << "ns"
            << " - " << end_time.GetNanoSeconds() << "ns), delta="
            << (end_time - start_time).GetNanoSeconds() << "ns");

        google::protobuf::uint32 txId = csi_response.csi(csi_i).tx_node().id();
        auto txPos = csi_response.csi(csi_i).tx_node().position();

        for (int rx_i=0; rx_i < csi_response.csi(csi_i).rx_nodes_size(); rx_i++) {
            Time delay = NanoSeconds(csi_response.csi(csi_i).rx_nodes(rx_i).delay());
            double wb_loss = csi_response.csi(csi_i).rx_nodes(rx_i).wb_loss();
            //Time ttl = NanoSeconds(csi_response.csi(csi_i).rx_nodes(rx_i).ttl());

            google::protobuf::uint32 rxId = csi_response.csi(csi_i).rx_nodes(rx_i).id();
            auto rxPos = csi_response.csi(csi_i).rx_nodes(rx_i).position();

            int num_ofdm_subcarrier = csi_response.csi(csi_i).rx_nodes(rx_i).csi_imag().size();

            NS_LOG_DEBUG("\t\t: Response (delay: " << delay << ", loss: " << wb_loss << ")"
              << " (TxId: " << txId << " [" << csi_response.csi(csi_i).tx_node().position().x()
              << "," << csi_response.csi(csi_i).tx_node().position().y() << "," << csi_response.csi(csi_i).tx_node().position().z() << "] -> "
              << rxId << " [" << csi_response.csi(csi_i).rx_nodes(rx_i).position().x() << "," << csi_response.csi(csi_i).rx_nodes(rx_i).position().y()
              << "," << csi_response.csi(csi_i).rx_nodes(rx_i).position().z() << ",NSC" << num_ofdm_subcarrier << "])");

            // Add the info from all other receivers to the cache
            CacheKey otherkey = CacheKey(txId, rxId);
            CacheEntry entry = CacheEntry(delay, wb_loss, start_time, end_time, num_ofdm_subcarrier,
                txId, rxId ,Vector(txPos.x(),txPos.y(),txPos.z()), Vector(rxPos.x(),rxPos.y(),rxPos.z()));

            // CFR: remove guards
            for (int i=0; i < num_ofdm_subcarrier; i++)
            {
                int freq = csi_response.csi(csi_i).rx_nodes(rx_i).frequencies(i);
                entry.m_freq.emplace_back(freq);

                double imag = csi_response.csi(csi_i).rx_nodes(rx_i).csi_imag(i);
                double real = csi_response.csi(csi_i).rx_nodes(rx_i).csi_real(i);
                entry.m_cfr.emplace_back(real, imag);
            }

            auto cache_it = m_cache.find(otherkey);

            if (cache_it != m_cache.end()) {
                // pair already known
                cache_it->second.push_back(entry);
            } else {
                // create new entry with zero vector
                m_cache.insert(std::make_pair(otherkey, std::vector<CacheEntry>()));
                cache_it = m_cache.find(otherkey);
                cache_it->second.push_back(entry);
            }
        }
    }

    // get result from cache
    CacheKey key2 = CacheKey(node_a->GetId(), node_b->GetId());
    auto cache_it = m_cache.find(key2);
    if (cache_it != m_cache.end())
    {
        // iterate over all stored entries for that link
        for (CacheEntry c_entry : cache_it->second) {
            // If delay and loss exist in the cache, check if the entry is not outdated
            if (c_entry.m_end_time >= current_time && c_entry.m_start_time <= current_time)
            {
                // Return cache entry as the value is still fresh
                return c_entry;
            }
        }
    }
    // cannot be reached
    CacheEntry dummy_entry = CacheEntry();
    return dummy_entry;
}

} // namespace ns3
