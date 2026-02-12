#include <ariac_components/penalty.hpp>

namespace ariac_components
{
  // --- Operator Overloads ---
  bool Penalty::operator==(const Penalty &_other) const
  {
    return (this->type == _other.type &&
            this->time == _other.time &&
            this->description == _other.description);
  }

  bool Penalty::equal(const ariac_components::Penalty &a, const ariac_components::Penalty &b)
  {
    return a == b;
  }

  // --- Serializer Implementation ---
  namespace serializers
  {
    std::ostream &PenaltySerializer::Serialize(std::ostream &_out, const Penalty &_penalty)
    {
      _out << static_cast<int>(_penalty.type) << " "
           << _penalty.time << " "
           << _penalty.description;
      return _out;
    }

    std::istream &PenaltySerializer::Deserialize(std::istream &_in, Penalty &_penalty)
    {
      int type_value = 0; // Initialize safely
      
      _in >> type_value;

      if (_in.fail()) return _in; // Stop if the int read failed

      _penalty.type = static_cast<PenaltyType>(type_value);

      // Now it's safe to read the rest
      _in >> _penalty.time >> _penalty.description;
      
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
using AriacPenalty = gz::sim::components::Penalty;
GZ_SIM_REGISTER_COMPONENT("ariac_components.Penalty", AriacPenalty)