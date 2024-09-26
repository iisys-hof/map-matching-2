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

#include "compare/comparator/comparator_forwarder_data.hpp"

namespace map_matching_2::compare {

    comparator_forwarder_data::comparator_forwarder_data(comparator &comparator_ref, compare::settings settings)
        : comparator_ref{comparator_ref}, settings{std::move(settings)} {}

    std::string comparator_forwarder_data::id(const multi_track_variant_type &multi_track_variant) {
        return std::visit([](auto &&multi_track) {
            return multi_track.id;
        }, multi_track_variant);
    }

}
