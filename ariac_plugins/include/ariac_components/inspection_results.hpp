#ifndef ARIAC_COMPONENTS_INSPECTION_RESULTS_HPP_
#define ARIAC_COMPONENTS_INSPECTION_RESULTS_HPP_

#include <iostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct InspectionResults
  {
    double avg_report_time = 0.0;
    int num_reports_submitted = 0;
    int num_correct_reports = 0;
    int num_correct_report_classifications = 0;

    bool operator==(const InspectionResults &_other) const;

    static bool equal(const ariac_components::InspectionResults &a, const ariac_components::InspectionResults &b);
  };

  namespace serializers
  {
    class InspectionResultsSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const InspectionResults &_inspectionResults);
        static std::istream &Deserialize(std::istream &_in, InspectionResults &_inspectionResults);
    };
  }
}

namespace gz::sim::components
{
  using InspectionResults = Component<ariac_components::InspectionResults,
                                      class InspectionResultsTag,
                                      ariac_components::serializers::InspectionResultsSerializer>;
}

#endif // ARIAC_COMPONENTS_INSPECTION_RESULTS_HPP_