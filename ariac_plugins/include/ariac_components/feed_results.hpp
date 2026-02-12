#ifndef ARIAC_COMPONENTS_FEED_RESULTS_HPP_
#define ARIAC_COMPONENTS_FEED_RESULTS_HPP_

#include <map>
#include <string>
#include <iostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

#include <ariac_interfaces/msg/cell_types.hpp>

namespace ariac_components
{
  struct FeedResults
  {
    std::map<int, int> cell_counts = {
      {ariac_interfaces::msg::CellTypes::LI_ION, 0},
      {ariac_interfaces::msg::CellTypes::NIMH, 0}
    };
    int num_defective = 0;

    bool operator==(const FeedResults &_other) const;

    static bool equal(const ariac_components::FeedResults &a, const ariac_components::FeedResults &b);
  };

  namespace serializers
  {
    class FeedResultsSerializer
    {
      public: 
        static std::ostream &Serialize(std::ostream &_out, const FeedResults &_feedResults);
        static std::istream &Deserialize(std::istream &_in, FeedResults &_feedResults);
    };
  }
}

namespace gz::sim::components
{
  using FeedResults = Component<ariac_components::FeedResults,
                                class FeedResultsTag,
                                ariac_components::serializers::FeedResultsSerializer>;
}

#endif // ARIAC_COMPONENTS_FEED_RESULTS_HPP_