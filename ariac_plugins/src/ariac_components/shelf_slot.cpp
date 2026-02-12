#include <ariac_components/shelf_slot.hpp>

namespace ariac_components
{
  // --- Operator Overloads ---
  bool ShelfSlot::operator==(const ShelfSlot &_other) const
  {
    return this->index == _other.index;
  }

  bool ShelfSlot::equal(const ShelfSlot &_a, const ShelfSlot &_b)
  {
    return _a == _b;
  }

  // --- Serializer Implementation ---
  namespace serializers
  {
    std::ostream &ShelfSlotSerializer::Serialize(std::ostream &_out, const ShelfSlot &_slot)
    {
      _out << _slot.index;
      return _out;
    }

    std::istream &ShelfSlotSerializer::Deserialize(std::istream &_in, ShelfSlot &_slot)
    {
      int val = 0;
      _in >> val;
      
      if (!_in.fail()) {
          _slot.index = val;
      }
      
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
using AriacShelfSlot = gz::sim::components::ShelfSlot;
GZ_SIM_REGISTER_COMPONENT("ariac_components.ShelfSlot", AriacShelfSlot)