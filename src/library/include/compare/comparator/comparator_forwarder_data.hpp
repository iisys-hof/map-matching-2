// Copyright (C) 2024 Adrian Wöltche
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see https://www.gnu.org/licenses/.

#ifndef MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_DATA_HPP
#define MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_DATA_HPP

#include <boost/unordered/unordered_flat_map.hpp>

#include "../comparator.hpp"
#include "../settings.hpp"

namespace map_matching_2::compare {

    struct comparator_forwarder_data {

        using multi_track_variant_type = comparator::multi_track_variant_type;
        using task_type = comparator::task_type;

        comparator_forwarder_data(comparator &comparator_ref, compare::settings settings);

        comparator &comparator_ref;
        const compare::settings settings;

        boost::unordered_flat_map<std::string, multi_track_variant_type> matches;
        boost::unordered_flat_map<std::string, multi_track_variant_type> compares;

        [[nodiscard]] static std::string id(const multi_track_variant_type &multi_track_variant);

    };

}

#endif //MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_DATA_HPP
