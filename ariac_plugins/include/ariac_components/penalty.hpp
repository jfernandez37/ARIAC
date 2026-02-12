#ifndef ARIAC_COMPONENTS_PENALTY_HPP_
#define ARIAC_COMPONENTS_PENALTY_HPP_

#include <string>
#include <iostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  enum class PenaltyType {
    GOOD_CELL_IN_INSPECTION_BIN,
    CELL_IN_CONVEYOR_BIN,
    OBJECT_ON_INVALID_SURFACE,
    AGV_COLLISION,
    ROBOT_COLLISION
  };

  struct Penalty
  {
    PenaltyType type;
    double time;
    std::string description;

    bool operator==(const Penalty &_other) const;

    static bool equal(const ariac_components::Penalty &a, const ariac_components::Penalty &b);
  };

  namespace serializers
  {
    class PenaltySerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const Penalty &_penalty);
        static std::istream &Deserialize(std::istream &_in, Penalty &_penalty);
    };
  }
}

namespace gz::sim::components
{
  using Penalty = Component<ariac_components::Penalty,
                            class PenaltyTag,
                            ariac_components::serializers::PenaltySerializer>;
}

#endif // ARIAC_COMPONENTS_PENALTY_HPP_