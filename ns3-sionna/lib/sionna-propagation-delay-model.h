/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#ifndef SIONNA_PROPAGATION_DELAY_MODEL_H
#define SIONNA_PROPAGATION_DELAY_MODEL_H

#include "sionna-propagation-cache.h"
#include "ns3/propagation-delay-model.h"

namespace ns3
{

/**
 * The propagation delay model: we compute the delay of the fastest path in
 * Sionna (LOS if available)
 */
class SionnaPropagationDelayModel : public PropagationDelayModel
{
    public:
        static TypeId GetTypeId();

        SionnaPropagationDelayModel();
        ~SionnaPropagationDelayModel() override;

        void SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache);
        Time GetDelay(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override;

    private:
        int64_t DoAssignStreams(int64_t stream) override;
        Ptr<SionnaPropagationCache> m_propagationCache;
};

} // namespace ns3

#endif // SIONNA_PROPAGATION_DELAY_MODEL_H