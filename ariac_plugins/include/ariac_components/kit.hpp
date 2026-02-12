#ifndef ARIAC_COMPONENTS_KIT_HPP_
#define ARIAC_COMPONENTS_KIT_HPP_

#include <map>
#include <optional>
#include <string>
#include <iostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct SlotCellInfo
  {
    int cell_type;
    bool defective;
    double voltage;

    bool operator==(const SlotCellInfo &_other) const;
  };

  struct Kit
  {
    // Fixed size map for 4 slots
    std::map<int, std::optional<SlotCellInfo>> slots = {
      {1, std::nullopt},
      {2, std::nullopt},
      {3, std::nullopt},
      {4, std::nullopt}
    };

    bool operator==(const Kit &_other) const;

    static bool equal(const Kit &_a, const Kit &_b);
  };

  namespace serializers
  {
    class KitSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const Kit &_kit);
        static std::istream &Deserialize(std::istream &_in, Kit &_kit);
    };
  }
}

namespace gz::sim::components
{
  using Kit = Component<ariac_components::Kit,
                        class KitTag,
                        ariac_components::serializers::KitSerializer>;
}

#endif // ARIAC_COMPONENTS_KIT_HPP_