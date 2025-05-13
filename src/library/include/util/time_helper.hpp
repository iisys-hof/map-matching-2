// Copyright (C) 2020-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_UTIL_TIME_HELPER_HPP
#define MAP_MATCHING_2_UTIL_TIME_HELPER_HPP

#include <chrono>

namespace map_matching_2::util {

    class time_helper {

    public:
        using clock_type = std::chrono::steady_clock;
        using time_point_type = clock_type::time_point;
        using duration_type = std::chrono::duration<double>;

        explicit time_helper();

        explicit time_helper(double max_time);

        explicit time_helper(duration_type max_duration);

        [[nodiscard]] constexpr const time_point_type &start() const {
            return _start;
        }

        [[nodiscard]] constexpr const time_point_type &current() const {
            return _current;
        }

        [[nodiscard]] constexpr const duration_type &max_duration() const {
            return _max_duration;
        }

        constexpr void set_max_duration(const duration_type &max_duration) {
            _max_duration = max_duration;
        }

        const time_point_type &update();

        [[nodiscard]] constexpr duration_type elapsed() const {
            return _current - _start;
        }

        [[nodiscard]] constexpr double elapsed_seconds() const {
            return elapsed().count();
        }

        [[nodiscard]] constexpr bool has_max_duration() const {
            return _max_duration > duration_type::zero();
        }

        [[nodiscard]] constexpr bool reached_max_duration() {
            if (not _has_reached) {
                _has_reached = has_max_duration() and elapsed() >= _max_duration;
            }
            return _has_reached;
        }

        [[nodiscard]] bool update_and_has_reached();

    private:
        const time_point_type _start;
        time_point_type _current;
        duration_type _max_duration;
        bool _has_reached{false};

    };

}

#endif //MAP_MATCHING_2_UTIL_TIME_HELPER_HPP
