#ifndef ARIAC_COMPONENTS_MODULE_HPP_
#define ARIAC_COMPONENTS_MODULE_HPP_

#include <map>
#include <string>
#include <iostream>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  enum class CellOrientation
  {
    UP,
    DOWN,
    NOT_PRESENT
  };

  struct Module
  {
    std::map<int, CellOrientation> cell_orientation = {
      {1, CellOrientation::NOT_PRESENT},
      {2, CellOrientation::NOT_PRESENT},
      {3, CellOrientation::NOT_PRESENT},
      {4, CellOrientation::NOT_PRESENT}
    };

    std::map<int, gz::sim::Entity> cell_entities = {
      {1, gz::sim::kNullEntity},
      {2, gz::sim::kNullEntity},
      {3, gz::sim::kNullEntity},
      {4, gz::sim::kNullEntity}
    };

    std::map<int, bool> top_welds = {
      {1, false},
      {2, false},
      {3, false},
      {4, false}
    };

    std::map<int, bool> bottom_welds = {
      {1, false},
      {2, false}
    };

    gz::sim::Entity bottom_shell_entity = gz::sim::kNullEntity;
    gz::sim::Entity top_shell_entity = gz::sim::kNullEntity;

    bool operator==(const Module &_other) const;

    static bool equal(const Module &_a, const Module &_b);
  };

  namespace serializers
  {
    class ModuleSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const Module &_module);
        static std::istream &Deserialize(std::istream &_in, Module &_module);
    };
  }
}

namespace gz::sim::components
{
  using Module = Component<ariac_components::Module,
                           class ModuleTag,
                           ariac_components::serializers::ModuleSerializer>;
}

#endif // ARIAC_COMPONENTS_MODULE_HPP_