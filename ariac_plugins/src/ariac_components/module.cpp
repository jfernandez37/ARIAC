#include <ariac_components/module.hpp>

namespace ariac_components
{
  // --- Operator Overloads ---
  bool Module::operator==(const Module &_other) const
  {
    return this->cell_orientation == _other.cell_orientation &&
           this->cell_entities == _other.cell_entities &&
           this->top_welds == _other.top_welds &&
           this->bottom_welds == _other.bottom_welds &&
           this->bottom_shell_entity == _other.bottom_shell_entity &&
           this->top_shell_entity == _other.top_shell_entity;
  }

  bool Module::equal(const Module &_a, const Module &_b)
  {
    return _a == _b;
  }

  // --- Serializer Implementation ---
  namespace serializers
  {
    std::ostream &ModuleSerializer::Serialize(std::ostream &_out, const Module &_module)
    {
      // 1. Cell Orientation (1-4)
      for (int i = 1; i <= 4; i++)
      {
        _out << static_cast<int>(_module.cell_orientation.at(i)) << " ";
      }

      // 2. Cell Entities (1-4)
      for (int i = 1; i <= 4; i++)
      {
        _out << _module.cell_entities.at(i) << " ";
      }

      // 3. Top Welds (1-4)
      for (int i = 1; i <= 4; i++)
      {
        _out << _module.top_welds.at(i) << " ";
      }

      // 4. Bottom Welds (1-2)
      for (int i = 1; i <= 2; i++)
      {
        _out << _module.bottom_welds.at(i) << " ";
      }

      // 5. Shell Entities
      _out << _module.bottom_shell_entity << " "
           << _module.top_shell_entity;

      return _out;
    }

    std::istream &ModuleSerializer::Deserialize(std::istream &_in, Module &_module)
    {
      // 1. Cell Orientation
      for (int i = 1; i <= 4; i++)
      {
        int orientation = 0; // Initialize safe default
        _in >> orientation;
        _module.cell_orientation[i] = static_cast<CellOrientation>(orientation);
      }

      // 2. Cell Entities
      for (int i = 1; i <= 4; i++)
      {
        _in >> _module.cell_entities[i];
      }

      // 3. Top Welds
      for (int i = 1; i <= 4; i++)
      {
        bool weld = false; 
        _in >> weld;
        _module.top_welds[i] = weld;
      }

      // 4. Bottom Welds
      for (int i = 1; i <= 2; i++)
      {
        bool weld = false;
        _in >> weld;
        _module.bottom_welds[i] = weld;
      }

      // 5. Shell Entities
      _in >> _module.bottom_shell_entity >> _module.top_shell_entity;

      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
using AriacModule = gz::sim::components::Module;
GZ_SIM_REGISTER_COMPONENT("ariac_components.Module", AriacModule)