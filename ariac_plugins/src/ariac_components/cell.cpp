#include <ariac_components/cell.hpp>

namespace ariac_components
{
  bool Cell::operator==(const Cell &_other) const
  {
    return (this->cell_name == _other.cell_name &&
            this->cell_type == _other.cell_type &&
            this->defective == _other.defective &&
            this->voltage == _other.voltage &&
            this->defect_type == _other.defect_type &&
            this->rotation == _other.rotation &&
            this->time_created == _other.time_created &&
            this->cell_entity == _other.cell_entity);
  }

  namespace serializers
  {
    std::ostream &CellSerializer::Serialize(std::ostream &_out, const Cell &_cell)
    {
      _out << _cell.cell_name << " "
           << _cell.cell_type << " "
           << _cell.defective << " "
           << _cell.voltage << " "
           << _cell.defect_type << " "
           << _cell.rotation << " "
           << _cell.time_created << " "
           << _cell.cell_entity;
      return _out;
    }

    std::istream &CellSerializer::Deserialize(std::istream &_in, Cell &_cell)
    {
      _in >> _cell.cell_name
          >> _cell.cell_type
          >> _cell.defective
          >> _cell.voltage
          >> _cell.defect_type
          >> _cell.rotation
          >> _cell.time_created
          >> _cell.cell_entity;
      
      // Although there are no vectors to crash here, 
      // it is good practice to return the stream state.
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
// 1. Create a local alias to handle the namespace "::" issue
using AriacCell = gz::sim::components::Cell;

// 2. Register the component
GZ_SIM_REGISTER_COMPONENT("ariac_components.Cell", AriacCell)