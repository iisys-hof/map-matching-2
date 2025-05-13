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

#include "util/time_helper.hpp"

namespace map_matching_2::util {


    time_helper::time_helper()
        : time_helper{duration_type::zero()} {}

    time_helper::time_helper(const double max_time)
        : time_helper{duration_type{max_time}} {}

    time_helper::time_helper(const duration_type max_duration)
        : _start{clock_type::now()}, _max_duration{max_duration} {
        _current = _start;
    }

    const time_helper::time_point_type &time_helper::update() {
        if (not _has_reached) {
            _current = clock_type::now();
        }
        return _current;
    }

    bool time_helper::update_and_has_reached() {
        if (not _has_reached) {
            update();
        }
        return reached_max_duration();
    }

}
