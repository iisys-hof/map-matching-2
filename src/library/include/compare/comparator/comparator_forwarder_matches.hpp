// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_MATCHES_HPP
#define MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_MATCHES_HPP

#include "comparator_forwarder_data.hpp"

namespace map_matching_2::compare {

    class comparator_forwarder_matches {

    public:
        using import_multi_track_variant_type = geometry::track::import_multi_track_variant_type;

        constexpr explicit comparator_forwarder_matches(comparator_forwarder_data &data)
            : _data{data} {}

        void pass(import_multi_track_variant_type multi_track) const;

    private:
        comparator_forwarder_data &_data;

    };

}

#endif //MAP_MATCHING_2_COMPARE_COMPARATOR_COMPARATOR_FORWARDER_MATCHES_HPP
