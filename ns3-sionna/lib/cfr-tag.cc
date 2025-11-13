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

#include "cfr-tag.h"
#include "ns3/log.h"
#include <iomanip>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("CFRTag");

TypeId
CFRTag::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::CFRTag")
      .SetParent<Tag> ()
      .SetGroupName("Sionna")
      .AddConstructor<CFRTag> ()
    ;
    return tid;
}

TypeId
CFRTag::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

CFRTag::CFRTag ()
  : Tag (), m_complexes ()
{
}

CFRTag::~CFRTag ()
{
}

void
CFRTag::SetComplexes (std::vector<std::complex<double>> complexes)
{
    m_complexes = complexes;
}

std::vector<std::complex<double>>
CFRTag::GetComplexes (void) const
{
    return m_complexes;
}

void
CFRTag::SetPathloss (double pathloss)
{
    m_pathloss = pathloss;
}

double
CFRTag::GetPathloss (void) const
{
    return m_pathloss;
}

void
CFRTag::Serialize (TagBuffer i) const
{
    // Serialize size first
    i.WriteU32 (m_complexes.size ());

    // Serialize each complex as real + imag doubles
    for (const auto& c : m_complexes) {
        i.WriteDouble (c.real ());
        i.WriteDouble (c.imag ());
    }

    // Write pathloss
    i.WriteDouble(m_pathloss);
}

void
CFRTag::Deserialize (TagBuffer i)
{
    // Deserialize size
    uint32_t size = i.ReadU32 ();

    // Resize and deserialize each complex
    m_complexes.resize (size);
    for (uint32_t j = 0; j < size; ++j) {
        double real = i.ReadDouble ();
        double imag = i.ReadDouble ();
        m_complexes[j] = std::complex<double> (real, imag);
    }
    m_pathloss = i.ReadDouble ();
}

uint32_t
CFRTag::GetSerializedSize () const
{
    // 4 bytes for size + 16 bytes per complex (2 doubles) + 8 for pathloss double
    return 4 + static_cast<uint32_t> (m_complexes.size ()) * 16 + 8;
}

void
CFRTag::Print (std::ostream &os) const
{
    os << "CFR=[";
    for (size_t i = 0; i < m_complexes.size (); ++i) {
        if (i > 0) os << ", ";
        os << "(" << std::fixed << std::setprecision(2)
           << m_complexes[i].real () << "+" << m_complexes[i].imag () << "j)";
    }
    os << "]";
    os << ", Pathloss=" << m_pathloss << "dB";
}

} // namespace ns3