/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#include "sionna-propagation-delay-model.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPropagationDelayModel");

NS_OBJECT_ENSURE_REGISTERED(SionnaPropagationDelayModel);

TypeId
SionnaPropagationDelayModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SionnaPropagationDelayModel")
            .SetParent<PropagationDelayModel>()
            .SetGroupName("Propagation")
            .AddConstructor<SionnaPropagationDelayModel>();
    return tid;
}

SionnaPropagationDelayModel::SionnaPropagationDelayModel()
    : m_propagationCache(nullptr)
{
}

SionnaPropagationDelayModel::~SionnaPropagationDelayModel()
{
}

void
SionnaPropagationDelayModel::SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache)
{
    m_propagationCache = propagationCache;
}

Time
SionnaPropagationDelayModel::GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
    NS_ASSERT_MSG(m_propagationCache, "SionnaPropagationDelayModel must have a SionnaPropagationCache.");
    return m_propagationCache->GetPropagationDelay(a, b);
}

int64_t
SionnaPropagationDelayModel::DoAssignStreams(int64_t stream)
{
    return 0;
}

} // namespace ns3