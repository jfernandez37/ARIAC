#ifndef ARIAC_COMPONENTS_TRIAL_HPP_
#define ARIAC_COMPONENTS_TRIAL_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct ConveyorMalfunction
  {
    int start_time;
    int duration;

    bool operator==(const ConveyorMalfunction &_other) const;
  };

  struct VacuumToolMalfunction
  {
    int tool;
    int grasp_occurrence;

    bool operator==(const VacuumToolMalfunction &_other) const;
  };

  struct VoltageTesterMalfunction
  {
    int tester;
    int start_time;
    int duration;

    bool operator==(const VoltageTesterMalfunction &_other) const;
  };

  struct Trial
  {
    std::string id;
    int seed;
    double defect_rate;
    int time_limit;
    int num_kits;
    int num_modules;

    std::vector<int> possible_defects;
    std::vector<ConveyorMalfunction> conveyor_malfunctions;
    std::vector<VacuumToolMalfunction> vacuum_tool_malfunctions;
    std::vector<VoltageTesterMalfunction> voltage_tester_malfunctions;

    bool operator==(const Trial &_other) const;

    static bool equal(const ariac_components::Trial &a, const ariac_components::Trial &b);
  };

  namespace serializers
  {
    class TrialSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const Trial &_trial);
        static std::istream &Deserialize(std::istream &_in, Trial &_trial);
    };
  }
}

// Component Registration Alias
namespace gz::sim::components
{
  using Trial = Component<ariac_components::Trial,
                          class TrialTag,
                          ariac_components::serializers::TrialSerializer>;
}

#endif // ARIAC_COMPONENTS_TRIAL_HPP_