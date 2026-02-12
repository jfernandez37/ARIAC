#ifndef ARIAC_COMPONENTS_CELL_HPP_
#define ARIAC_COMPONENTS_CELL_HPP_

#include <string>
#include <iostream>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct Cell
  {
    std::string cell_name;
    int cell_type;
    bool defective;
    double voltage;
    int defect_type;
    double rotation;
    double time_created;
    gz::sim::Entity cell_entity;

    bool operator==(const Cell &_other) const;
  };

  namespace serializers
  {
    class CellSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const Cell &_cell);
        static std::istream &Deserialize(std::istream &_in, Cell &_cell);
    };
  }
}

namespace gz::sim::components
{
  using Cell = Component<ariac_components::Cell,
                         class CellTag,
                         ariac_components::serializers::CellSerializer>;
}

#endif // ARIAC_COMPONENTS_CELL_HPP_