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

#include "compare/comparator/comparator_forwarder_data.hpp"

namespace map_matching_2::compare {

    comparator_forwarder_data::comparator_forwarder_data(comparator &comparator_ref, compare::settings settings)
        : comparator_ref{comparator_ref}, settings{std::move(settings)} {}

    std::string comparator_forwarder_data::id(const multi_track_variant_type &multi_track_variant) {
        return std::visit([](auto &&multi_track) {
            return multi_track.id;
        }, multi_track_variant);
    }

    void comparator_forwarder_data::finish() {
        if (not settings.ignore_non_existent) {
            // for every ground truth that has no match, compare to an empty "non-existing" match
            for (const auto &compare_entry : compares) {
                const auto &id = compare_entry.first;
                const auto &multi_track = compare_entry.second;

                std::visit([this, &id]<typename MultiTrack>(const MultiTrack &ground_truth) {
                    using multi_track_type = std::remove_reference_t<MultiTrack>;
                    using multi_rich_line_type = typename multi_track_type::multi_rich_line_type;
                    using point_type = typename multi_rich_line_type::point_type;

                    geometry::track::multi_track_type<point_type> empty_match{id, multi_rich_line_type{}};

                    comparator_ref.compare(std::move(empty_match), ground_truth, settings);
                }, multi_track);
            }
        }
    }

}
