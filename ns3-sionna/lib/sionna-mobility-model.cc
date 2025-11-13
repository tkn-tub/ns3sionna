/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-mobility-model.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaMobilityModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaMobilityModel);

TypeId
SionnaMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<SionnaMobilityModel>()
            .AddAttribute("Model",
                          "The model indicates whether ConstantPositionMobilityModel "
                          "or RandomWalk2dMobilityModel is used",
                          EnumValue(SionnaMobilityModel::MODEL_CONSTANT_POSITION),
                          MakeEnumAccessor(&SionnaMobilityModel::m_model),
                          MakeEnumChecker(SionnaMobilityModel::MODEL_CONSTANT_POSITION,
                                          "Constant Position",
                                          SionnaMobilityModel::MODEL_RANDOM_WALK,
                                          "Random Walk"))
            .AddAttribute("Mode",
                          "The mode indicates the condition used to "
                          "change the current speed and direction",
                          EnumValue(SionnaMobilityModel::MODE_DISTANCE),
                          MakeEnumAccessor(&SionnaMobilityModel::m_mode),
                          MakeEnumChecker(SionnaMobilityModel::MODE_DISTANCE,
                                          "Distance",
                                          SionnaMobilityModel::MODE_TIME,
                                          "Time"))
            .AddAttribute("Time",
                          "Change current direction and speed after moving for this delay.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&SionnaMobilityModel::m_modeTime),
                          MakeTimeChecker())
            .AddAttribute("Distance",
                          "Change current direction and speed after moving for this distance.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&SionnaMobilityModel::m_modeDistance),
                          MakeDoubleChecker<double>())
            .AddAttribute("Speed",
                          "A random variable used to pick the speed (m/s).",
                          StringValue("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),
                          MakePointerAccessor(&SionnaMobilityModel::m_speed),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Direction",
                          "A random variable used to pick the direction (radians).",
                          StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                          MakePointerAccessor(&SionnaMobilityModel::m_direction),
                          MakePointerChecker<RandomVariableStream>());
    return tid;
}

SionnaMobilityModel::SionnaMobilityModel()
{
}

SionnaMobilityModel::~SionnaMobilityModel()
{
}

std::string
SionnaMobilityModel::GetModel() const
{
    if (m_model == SionnaMobilityModel::MODEL_RANDOM_WALK)
    {
        return "Random Walk";
    }
    else
    {
        return "Constant Position";
    }
}

std::string
SionnaMobilityModel::GetMode() const
{
    if (m_mode == SionnaMobilityModel::MODE_TIME)
    {
        return "Time";
    }
    else
    {
        return "Distance";
    }
}

double
SionnaMobilityModel::GetModeDistance() const
{
    return m_modeDistance;
}

Time
SionnaMobilityModel::GetModeTime() const
{
    return m_modeTime;
}

Ptr<RandomVariableStream>
SionnaMobilityModel::GetSpeed() const
{
    return m_speed;
}

Ptr<RandomVariableStream>
SionnaMobilityModel::GetDirection() const
{
    return m_direction;
}

Vector
SionnaMobilityModel::DoGetPosition() const
{
    return m_position;
}

void
SionnaMobilityModel::DoSetPosition(const Vector& position)
{
    m_position = position;
}

Vector
SionnaMobilityModel::DoGetVelocity() const
{
    return Vector(0.0, 0.0, 0.0);
}

/*
void
SionnaMobilityModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}
*/
} // namespace ns3
