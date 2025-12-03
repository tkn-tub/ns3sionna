/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-helper.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "../model/sionna-mobility-model.h"
#include "sionna-utils.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaHelper");

SionnaHelper::SionnaHelper(std::string environment, std::string zmq_url): m_zmq_url(zmq_url),
    m_environment(environment), m_zmq_context(1), m_zmq_socket(m_zmq_context, ZMQ_REQ)
{
    // Connect
    m_zmq_socket.connect(zmq_url);
    m_mode = MODE_P2MP_LAH;
    m_sub_mode = 1;
    // WiFi 6
    m_frequency = 5210;
    SetChannelBandwidth(80 * 3);
    m_fft_size = 1024 * 3;
    m_subcarrier_spacing = 78125;
    m_min_coherence_time_ms = 100000; // min coherence time is 100s
    m_time_evo_model = "position";

    std::cout << "Env: " << m_environment << std::endl;
}

SionnaHelper::~SionnaHelper()
{
}

void
SionnaHelper::SetFrequency(int frequency)
{
    m_frequency = frequency;
}

int
SionnaHelper::GetFrequency()
{
    return m_frequency;
}

void
SionnaHelper::SetChannelBandwidth(int channel_bw)
{
    m_channel_bw = channel_bw;

    // update noise floor
    static const double BOLTZMANN = 1.3803e-23;
    // Nt is the power of thermal noise in W
    double Nt = BOLTZMANN * 293 * channel_bw;
    // receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
    double m_noiseFigure = 5;
    double noiseFloor = m_noiseFigure * Nt;
    double noise = noiseFloor;

    m_noiseDbm = 10 * std::log10(noise/1e-3);
    //std::cout << "Noise floor for optimize: " << m_noiseDbm << " dBm" << std::endl;
}

void
SionnaHelper::SetFFTSize(int fft_size)
{
    m_fft_size = fft_size;
}

void
SionnaHelper::SetSubcarrierSpacing(int subcarrier_spacing)
{
    m_subcarrier_spacing = subcarrier_spacing;
}

void
SionnaHelper::SetMode(int mode)
{
    m_mode = mode;
}

void
SionnaHelper::SetSubMode(int sub_mode)
{
    m_sub_mode = sub_mode;
}

void
SionnaHelper::Configure(int frequency, int channel_bw, int fft_size, int ofdm_subcarrier_spacing, int min_coherence_time_ms)
{
    m_min_coherence_time_ms = min_coherence_time_ms;
    Configure(frequency, channel_bw, fft_size, ofdm_subcarrier_spacing);
}

void
SionnaHelper::Configure(int frequency, int channel_bw, int fft_size, int ofdm_subcarrier_spacing)
{
    NS_ASSERT_MSG(frequency >= 0, "Center frequency must be positive");
    NS_ASSERT_MSG(channel_bw >= 0 && channel_bw <= 10000, "Channel bandwidth must be between 0 and 10000 MHz");
    NS_ASSERT_MSG(fft_size >= 0, "FFT size must be positive");
    NS_ASSERT_MSG(ofdm_subcarrier_spacing >= 0, "OFDM subcarrier spacing must be positive");

    SetFrequency(frequency);
    SetChannelBandwidth(channel_bw * GUARD_MULTIPLIER); // effective channel bandwidth is 3x due to guard bands
    SetFFTSize(fft_size * GUARD_MULTIPLIER);
    SetSubcarrierSpacing(ofdm_subcarrier_spacing);

    std::cout << "ns3sionna configured with fc= " << frequency << "MHz, B=" << m_channel_bw
        << "MHz" << ", FFT=" << m_fft_size << ", Evo model:" << m_time_evo_model
        << ", MinTc=" << m_min_coherence_time_ms << "ms" << std::endl;
}

double
SionnaHelper::GetNoiseFloor()
{
    return m_noiseDbm;
}

void
SionnaHelper::RandomVariableStreamMessage(ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream* message,
                            Ptr<RandomVariableStream> random_variable)
{
    std::string random_variable_name = random_variable->GetInstanceTypeId().GetName();
    if (random_variable_name == "ns3::UniformRandomVariable")
    {
        ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream::Uniform* uniform = message->mutable_uniform();
        Ptr<UniformRandomVariable> uniform_variable = DynamicCast<UniformRandomVariable>(random_variable);
        uniform->set_min(uniform_variable->GetMin());
        uniform->set_max(uniform_variable->GetMax());
    }
    else if (random_variable_name == "ns3::ConstantRandomVariable")
    {
        ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream::Constant* constant = message->mutable_constant();
        Ptr<ConstantRandomVariable> constant_variable = DynamicCast<ConstantRandomVariable>(random_variable);
        constant->set_value(constant_variable->GetConstant());
    }
    else if (random_variable_name == "ns3::NormalRandomVariable")
    {
        ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream::Normal* normal = message->mutable_normal();
        Ptr<NormalRandomVariable> normal_variable = DynamicCast<NormalRandomVariable>(random_variable);
        normal->set_mean(normal_variable->GetMean());
        normal->set_variance(normal_variable->GetVariance());
    }
    else
    {
        NS_FATAL_ERROR("RandomVariableStream must be Uniform, Constant, or Normal.");
    }
}

void
SionnaHelper::Start()
{
    std::cout << "ns3sionna configured for mode: " << m_mode << ", submode: " << m_sub_mode << std::endl;

    std::cout << "ns3sionna: trying to connect to sionna via " << m_zmq_url << std::endl;

    // Prepare the information message
    ns3sionna::Wrapper wrapper;

    // Fill the information message
    ns3sionna::SimInitMessage* simulation_info = wrapper.mutable_sim_init_msg();
    simulation_info->set_scene_fname(m_environment);
    simulation_info->set_seed(RngSeedManager::GetSeed());
    simulation_info->set_frequency(m_frequency);
    simulation_info->set_channel_bw(m_channel_bw);
    simulation_info->set_fft_size(m_fft_size);
    simulation_info->set_min_coherence_time_ms(m_min_coherence_time_ms);
    simulation_info->set_time_evo_model(m_time_evo_model);
    simulation_info->set_subcarrier_spacing(m_subcarrier_spacing);
    simulation_info->set_mode(m_mode);
    simulation_info->set_sub_mode(m_sub_mode);

    NodeContainer c = NodeContainer::GetGlobal();
    for (auto iter = c.Begin(); iter != c.End(); ++iter)
    {
        Ptr<MobilityModel> mobilityModel = (*iter)->GetObject<MobilityModel>();
        
        if (mobilityModel)
        {
            // Only send node information if the node has the SionnaMobilityModel
            Ptr<SionnaMobilityModel> sionnaMobilityModel = DynamicCast<SionnaMobilityModel>(mobilityModel);
            NS_ASSERT_MSG(sionnaMobilityModel, "Not using SionnaMobilityModel.");
            
            ns3sionna::SimInitMessage::NodeInfo* node_info = simulation_info->add_nodes();
            node_info->set_id((*iter)->GetId());
            
            Vector position = mobilityModel->GetPosition();

            if (sionnaMobilityModel->GetModel() == "Random Walk")
            {
                ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel* random_walk_model = node_info->mutable_random_walk_model();
                
                ns3sionna::SimInitMessage::NodeInfo::Vector* position_vector = random_walk_model->mutable_position();
                position_vector->set_x(position.x);
                position_vector->set_y(position.y);
                position_vector->set_z(position.z);

                if (sionnaMobilityModel->GetMode() == "Wall")
                {
                    bool wall_value = sionnaMobilityModel->GetModeWall();
                    random_walk_model->set_wall_value(wall_value);
                } else if (sionnaMobilityModel->GetMode() == "Time")
                {
                    int64_t time_value = sionnaMobilityModel->GetModeTime().GetNanoSeconds();
                    NS_ASSERT_MSG(time_value > 0, "Time value must be greater than 0 seconds.");
                    random_walk_model->set_time_value(time_value);
                }
                else
                {
                    double distance_value = sionnaMobilityModel->GetModeDistance();
                    NS_ASSERT_MSG(distance_value > 0.0, "Distance value must be greater than 0 meters.");
                    random_walk_model->set_distance_value(distance_value);
                }

                ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream* speed = random_walk_model->mutable_speed();
                RandomVariableStreamMessage(speed, sionnaMobilityModel->GetSpeed());
                
                ns3sionna::SimInitMessage::NodeInfo::RandomWalkModel::RandomVariableStream* direction = random_walk_model->mutable_direction();
                RandomVariableStreamMessage(direction, sionnaMobilityModel->GetDirection());
            }
            else
            {
                ns3sionna::SimInitMessage::NodeInfo::ConstantPositionModel* constant_position_model = node_info->mutable_constant_position_model();
                
                ns3sionna::SimInitMessage::NodeInfo::Vector* position_vector = constant_position_model->mutable_position();
                position_vector->set_x(position.x);
                position_vector->set_y(position.y);
                position_vector->set_z(position.z);
            }
        }
    }

    // Serialize the information message
    std::string serialized_message;
    wrapper.SerializeToString(&serialized_message);

    // Send the information message
    zmq::message_t zmq_message(serialized_message.data(), serialized_message.size());
    m_zmq_socket.send(zmq_message, zmq::send_flags::none);

    // Receive the reply message
    zmq::message_t zmq_reply;
    zmq::recv_result_t result = m_zmq_socket.recv(zmq_reply, zmq::recv_flags::none);
    
    NS_ASSERT_MSG(result, "Failed to receive reply after simulation information message.");
    
    // Check if the reply message is an ack
    ns3sionna::Wrapper reply_wrapper;
    reply_wrapper.ParseFromArray(zmq_reply.data(), zmq_reply.size());

    if (reply_wrapper.has_sim_ack())
    {
        if (reply_wrapper.sim_ack().no_error()) {
            std::cout << "ns3sionna: connection ... OK" << std::endl;
        } else
        {
            NS_ABORT_MSG("ns3sionna: connection ... FAILED with error: " + reply_wrapper.sim_ack().error_msg());
        }
    } else
    {
        NS_ABORT_MSG("ns3sionna: connection ... FAILED");
    }
}

void
SionnaHelper::Destroy()
{
    // Prepare the request message
    ns3sionna::Wrapper wrapper;
    wrapper.mutable_sim_close_request();
    
    // Serialize the request message
    std::string serialized_message;
    wrapper.SerializeToString(&serialized_message);

    // Send the request message
    zmq::message_t zmq_message(serialized_message.data(), serialized_message.size());
    m_zmq_socket.send(zmq_message, zmq::send_flags::none);

    // Receive the reply message
    zmq::message_t zmq_reply;
    zmq::recv_result_t result = m_zmq_socket.recv(zmq_reply, zmq::recv_flags::none);
    
    NS_ASSERT_MSG(result, "Failed to receive reply after close request message.");
    
    // Check if the reply message is an ack
    ns3sionna::Wrapper reply_wrapper;
    reply_wrapper.ParseFromArray(zmq_reply.data(), zmq_reply.size());
    
    NS_ASSERT_MSG(reply_wrapper.has_sim_ack(), "Reply after close request is not an ack.");

    // Close socket
    m_zmq_socket.close();
    std::cout << "ns3sionna socket closed" << std::endl;
}

} // namespace ns3
