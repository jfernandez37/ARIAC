#include <ariac_components/trial.hpp>

namespace ariac_components
{
  // --- Operator Overloads Implementation ---
  bool ConveyorMalfunction::operator==(const ConveyorMalfunction &_other) const
  {
    return (this->start_time == _other.start_time &&
            this->duration == _other.duration);
  }

  bool VacuumToolMalfunction::operator==(const VacuumToolMalfunction &_other) const
  {
    return (this->grasp_occurrence == _other.grasp_occurrence &&
            this->tool == _other.tool);
  }

  bool VoltageTesterMalfunction::operator==(const VoltageTesterMalfunction &_other) const
  {
    return (this->tester == _other.tester &&
            this->start_time == _other.start_time &&
            this->duration == _other.duration);
  }

  bool Trial::operator==(const Trial &_other) const
  {
    return (this->id == _other.id &&
            this->seed == _other.seed &&
            this->defect_rate == _other.defect_rate &&
            this->time_limit == _other.time_limit &&
            this->num_kits == _other.num_kits &&
            this->num_modules == _other.num_modules &&
            this->possible_defects == _other.possible_defects &&
            this->conveyor_malfunctions == _other.conveyor_malfunctions &&
            this->vacuum_tool_malfunctions == _other.vacuum_tool_malfunctions &&
            this->voltage_tester_malfunctions == _other.voltage_tester_malfunctions);
  }

  bool Trial::equal(const ariac_components::Trial &a, const ariac_components::Trial &b)
  {
    return a == b;
  }

  // --- Serializer Implementation ---
  namespace serializers
  {
    std::ostream &TrialSerializer::Serialize(std::ostream &_out, const Trial &_trial)
    {
      _out << _trial.id << " "
           << _trial.seed << " "
           << _trial.defect_rate << " "
           << _trial.time_limit << " "
           << _trial.num_kits << " "
           << _trial.num_modules << " ";

      // Serialize possible_defects
      _out << _trial.possible_defects.size() << " ";
      for (const auto &defect : _trial.possible_defects)
        _out << defect << " ";

      // Serialize conveyor_malfunctions
      _out << _trial.conveyor_malfunctions.size() << " ";
      for (const auto &malfunction : _trial.conveyor_malfunctions)
        _out << malfunction.start_time << " " << malfunction.duration << " ";

      // Serialize vacuum_tool_malfunctions
      _out << _trial.vacuum_tool_malfunctions.size() << " ";
      for (const auto &malfunction : _trial.vacuum_tool_malfunctions)
        _out << malfunction.tool << " " << malfunction.grasp_occurrence << " ";

      // Serialize voltage_tester_malfunctions
      _out << _trial.voltage_tester_malfunctions.size() << " ";
      for (const auto &malfunction : _trial.voltage_tester_malfunctions)
        _out << malfunction.tester << " " << malfunction.start_time << " " << malfunction.duration << " ";

      return _out;
    }

    std::istream &TrialSerializer::Deserialize(std::istream &_in, Trial &_trial)
    {
      _in >> _trial.id
          >> _trial.seed
          >> _trial.defect_rate
          >> _trial.time_limit
          >> _trial.num_kits
          >> _trial.num_modules;

      if (_in.fail()) return _in; // Basic safety check

      // Helper lambda to safely read vectors without crashing
      // This prevents "vector::reserve" errors from garbage memory
      auto safe_reserve = [&](size_t size, auto& vec) {
          vec.clear();
          if (size < 100000) { // Sanity check: Don't allocate if size is crazy
              vec.reserve(size);
          }
      };

      // 1. Possible Defects
      size_t defects_size = 0; // Initialize to 0!
      if (!(_in >> defects_size)) return _in;
      
      safe_reserve(defects_size, _trial.possible_defects);
      for (size_t i = 0; i < defects_size; ++i)
      {
        int defect;
        _in >> defect;
        _trial.possible_defects.push_back(defect);
      }

      // 2. Conveyor Malfunctions
      size_t conveyor_size = 0; // Initialize to 0!
      if (!(_in >> conveyor_size)) return _in;

      safe_reserve(conveyor_size, _trial.conveyor_malfunctions);
      for (size_t i = 0; i < conveyor_size; ++i)
      {
        ConveyorMalfunction malfunction;
        _in >> malfunction.start_time >> malfunction.duration;
        _trial.conveyor_malfunctions.push_back(malfunction);
      }

      // 3. Vacuum Tool Malfunctions
      size_t vacuum_size = 0; // Initialize to 0!
      if (!(_in >> vacuum_size)) return _in;

      safe_reserve(vacuum_size, _trial.vacuum_tool_malfunctions);
      for (size_t i = 0; i < vacuum_size; ++i)
      {
        VacuumToolMalfunction malfunction;
        _in >> malfunction.tool >> malfunction.grasp_occurrence;
        _trial.vacuum_tool_malfunctions.push_back(malfunction);
      }

      // 4. Voltage Tester Malfunctions
      size_t voltage_size = 0; // Initialize to 0!
      if (!(_in >> voltage_size)) return _in;

      safe_reserve(voltage_size, _trial.voltage_tester_malfunctions);
      for (size_t i = 0; i < voltage_size; ++i)
      {
        VoltageTesterMalfunction malfunction;
        _in >> malfunction.tester >> malfunction.start_time >> malfunction.duration;
        _trial.voltage_tester_malfunctions.push_back(malfunction);
      }

      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION (Must be in .cpp) ---
// This registers the component with Gazebo's factory.
// Because it is in the .cpp, it is only defined once.
using AriacTrial = gz::sim::components::Trial;
GZ_SIM_REGISTER_COMPONENT("ariac_components.Trial", AriacTrial)