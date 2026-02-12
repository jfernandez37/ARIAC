#include <ariac_components/kit.hpp>

namespace ariac_components
{
  // --- Operator Overloads ---
  bool SlotCellInfo::operator==(const SlotCellInfo &_other) const
  {
    return this->cell_type == _other.cell_type &&
           this->defective == _other.defective &&
           this->voltage == _other.voltage;
  }

  bool Kit::operator==(const Kit &_other) const
  {
    for (int i = 1; i <= 4; i++)
    {
      // 1. Check if both have value or both represent null
      if (this->slots.at(i).has_value() != _other.slots.at(i).has_value())
      {
        return false;
      }

      // 2. If both have value, check the content
      if (this->slots.at(i).has_value())
      {
        // Dereference optional to compare the SlotCellInfo inside
        if (!(*this->slots.at(i) == *_other.slots.at(i)))
        {
          return false;
        }
      }
    }
    return true;
  }

  bool Kit::equal(const Kit &_a, const Kit &_b)
  {
    return _a == _b;
  }

  // --- Serializer Implementation ---
  namespace serializers
  {
    std::ostream &KitSerializer::Serialize(std::ostream &_out, const Kit &_kit)
    {
      for (int i = 1; i <= 4; i++)
      {
        // Safety check: Ensure the key exists (it should for 1-4)
        if (_kit.slots.find(i) == _kit.slots.end()) {
             _out << "0 "; // Treat missing key as empty slot
             continue;
        }

        const auto& slot = _kit.slots.at(i);
        _out << slot.has_value() << " ";
        
        if (slot.has_value())
        {
          _out << slot->cell_type << " "
               << slot->defective << " "
               << slot->voltage << " ";
        }
      }
      return _out;
    }

    std::istream &KitSerializer::Deserialize(std::istream &_in, Kit &_kit)
    {
      for (int i = 1; i <= 4; i++)
      {
        // Initialize to false so failure doesn't trigger a read
        bool has_value = false;
        _in >> has_value;

        if (_in.fail()) break; // Stop if stream is bad

        if (has_value)
        {
          SlotCellInfo info;
          _in >> info.cell_type >> info.defective >> info.voltage;
          _kit.slots[i] = info;
        }
        else
        {
          _kit.slots[i] = std::nullopt;
        }
      }
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
using AriacKit = gz::sim::components::Kit;
GZ_SIM_REGISTER_COMPONENT("ariac_components.Kit", AriacKit)