#include <ariac_components/feed_results.hpp>

namespace ariac_components
{
  bool FeedResults::operator==(const FeedResults &_other) const
  {
    return (this->cell_counts == _other.cell_counts &&
            this->num_defective == _other.num_defective);
  }

  bool FeedResults::equal(const ariac_components::FeedResults &a, const ariac_components::FeedResults &b)
  {
    return a == b;
  }

  namespace serializers
  {
    std::ostream &FeedResultsSerializer::Serialize(std::ostream &_out, const FeedResults &_feedResults)
    {
      _out << _feedResults.cell_counts.size() << " ";
      for (const auto &[key, value] : _feedResults.cell_counts)
      {
        _out << key << " " << value << " ";
      }
      _out << _feedResults.num_defective;
      return _out;
    }

    std::istream &FeedResultsSerializer::Deserialize(std::istream &_in, FeedResults &_feedResults)
    {
      // 1. Initialize to 0 to prevent crashes
      size_t map_size = 0;
      
      // 2. Read size and check for failure
      if (!(_in >> map_size)) return _in;

      _feedResults.cell_counts.clear();

      // 3. Sanity Check: If the map size is impossibly large (garbage data), stop.
      if (map_size > 100000) {
          return _in;
      }

      for (size_t i = 0; i < map_size; ++i)
      {
        int key = 0, value = 0;
        // Check if read succeeds before inserting
        if (!(_in >> key >> value)) break;
        
        _feedResults.cell_counts[key] = value;
      }

      _in >> _feedResults.num_defective;
      return _in;
    }
  }
}

// --- COMPONENT REGISTRATION ---
// 1. Alias to handle namespace characters
using AriacFeedResults = gz::sim::components::FeedResults;

// 2. Register
GZ_SIM_REGISTER_COMPONENT("ariac_components.FeedResults", AriacFeedResults)