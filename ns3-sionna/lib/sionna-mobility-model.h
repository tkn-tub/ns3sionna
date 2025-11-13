/*
 * Copyright (c) 2024 Yannik Pilz
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Yannik Pilz <y.pilz@campus.tu-berlin.de>
 */

#ifndef SIONNA_MOBILITY_MODEL_H
#define SIONNA_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{

/**
 * The mobility models available in ns3sionna:
 * - constant position
 * - random walk
 *
 * Note: mobility is simulated inside Sionna and propagated back no ns3.
 */
class SionnaMobilityModel : public MobilityModel
{
    public:
        static TypeId GetTypeId();

        SionnaMobilityModel();
        ~SionnaMobilityModel() override;

        enum Model
        {
            MODEL_CONSTANT_POSITION,
            MODEL_RANDOM_WALK
        };

        enum Mode
        {
            MODE_DISTANCE,
            MODE_TIME
        };

        std::string GetModel() const;

        std::string GetMode() const;

        double GetModeDistance() const;

        Time GetModeTime() const;

        Ptr<RandomVariableStream> GetSpeed() const;

        Ptr<RandomVariableStream> GetDirection() const;

        //void SetPropagationCache(Ptr<SionnaPropagationCache> propagationCache);

    private:
        Vector DoGetPosition() const override;

        void DoSetPosition(const Vector& position) override;

        Vector DoGetVelocity() const override;

        Model m_model;
        Vector m_position;
        Mode m_mode;
        double m_modeDistance;
        Time m_modeTime;
        Ptr<RandomVariableStream> m_speed;
        Ptr<RandomVariableStream> m_direction;

        //Ptr<SionnaPropagationCache> m_propagationCache;
};

} // namespace ns3

#endif // SIONNA_MOBILITY_MODEL_H