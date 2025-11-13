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

#ifndef CFR_TAG_H
#define CFR_TAG_H

#include "ns3/tag.h"
#include "ns3/simple-ref-count.h"
#include <vector>      // For std::vector
#include <complex>     // For std::complex<double>

namespace ns3 {

/**
 * Tag used to deliver pathloss and channel state information (channel frequency response)
 * to application layer.
 */
class CFRTag : public Tag
{
public:
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;

    CFRTag ();
    virtual ~CFRTag ();

    // Set the vector of complex numbers (e.g., CFR coefficients)
    void SetComplexes (std::vector<std::complex<double>> complexes);
    // Get the vector of complex numbers
    std::vector<std::complex<double>> GetComplexes (void) const;

    void SetPathloss (double pathloss);
    double GetPathloss (void) const;

    // Tag interface methods
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual uint32_t GetSerializedSize () const;
    virtual void Print (std::ostream &os) const;

private:
    std::vector<std::complex<double>> m_complexes;  // complex CFR per OFDM subcarriers
    double m_pathloss; // the propagation pathloss
};

} // namespace ns3

#endif /* CFR_TAG_H */