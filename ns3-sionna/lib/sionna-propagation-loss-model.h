/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#ifndef SIONNA_PROPAGATION_LOSS_MODEL_H
#define SIONNA_PROPAGATION_LOSS_MODEL_H

#include "sionna-propagation-cache.h"

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * The propagation loss model: we compute the average loss (over all subcarriers) in Sionna
 */
class SionnaPropagationLossModel : public PropagationLossModel
{
    public:
        static TypeId GetTypeId();

        SionnaPropagationLossModel();
        ~SionnaPropagationLossModel() override;

        SionnaPropagationLossModel(const SionnaPropagationLossModel&) = delete;
        SionnaPropagationLossModel& operator=(const SionnaPropagationLossModel&) = delete;

        void SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache);

    private:
        double DoCalcRxPower(double txPowerDbm,
                             Ptr<MobilityModel> a,
                             Ptr<MobilityModel> b) const override;

        int64_t DoAssignStreams(int64_t stream) override;

        Ptr<SionnaPropagationCache> m_propagationCache;
};

} // namespace ns3

#endif // SIONNA_PROPAGATION_LOSS_MODEL_H