#include <ariac_components/inspection_results.hpp>

namespace ariac_components
{
  bool InspectionResults::operator==(const InspectionResults &_other) const
  {
    return (this->avg_report_time == _other.avg_report_time &&
            this->num_reports_submitted == _other.num_reports_submitted &&
            this->num_correct_reports == _other.num_correct_reports &&
            this->num_correct_report_classifications == _other.num_correct_report_classifications);
  }

  bool InspectionResults::equal(const ariac_components::InspectionResults &a, const ariac_components::InspectionResults &b)
  {
    return a == b;
  }

  namespace serializers
  {
    std::ostream &InspectionResultsSerializer::Serialize(std::ostream &_out, const InspectionResults &_inspectionResults)
    {
      _out << _inspectionResults.avg_report_time << " "
           << _inspectionResults.num_reports_submitted << " "
           << _inspectionResults.num_correct_reports << " "
           << _inspectionResults.num_correct_report_classifications;
      return _out;
    }

    std::istream &InspectionResultsSerializer::Deserialize(std::istream &_in, InspectionResults &_inspectionResults)
    {
      _in >> _inspectionResults.avg_report_time
          >> _inspectionResults.num_reports_submitted
          >> _inspectionResults.num_correct_reports
          >> _inspectionResults.num_correct_report_classifications;
      
      // Standard safety return
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
// 1. Alias to handle namespace characters
using AriacInspectionResults = gz::sim::components::InspectionResults;

// 2. Register
GZ_SIM_REGISTER_COMPONENT("ariac_components.InspectionResults", AriacInspectionResults)