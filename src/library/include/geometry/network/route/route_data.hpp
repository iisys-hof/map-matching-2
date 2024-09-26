// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP

#include <utility>

#include "geometry/rich_type/common.hpp"

#include "geometry/common.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/azimuth.hpp"
#include "geometry/algorithm/direction.hpp"

#include "concepts.hpp"

namespace map_matching_2::geometry::network {

    template<typename Length, typename Angle>
    class route_data {

    public:
        using length_type = Length;
        using angle_type = Angle;

        constexpr route_data(const route_data &other)
            : _length{other._length}, _azimuth{other._azimuth}, _directions{other._directions},
            _absolute_directions{other._absolute_directions}, _is_connected{other._is_connected} {}

        constexpr route_data(route_data &&other) noexcept
            : _length{std::move(other._length)}, _azimuth{std::move(other._azimuth)},
            _directions{std::move(other._directions)},
            _absolute_directions{std::move(other._absolute_directions)},
            _is_connected{std::move(other._is_connected)} {}

        constexpr route_data &operator=(const route_data &other) {
            if (this != &other) {
                _length = other._length;
                _azimuth = other._azimuth;
                _directions = other._directions;
                _absolute_directions = other._absolute_directions;
                _is_connected = other._is_connected;
            }
            return *this;
        }

        constexpr route_data &operator=(route_data &&other) noexcept {
            if (this != &other) {
                _length = std::move(other._length);
                _azimuth = std::move(other._azimuth);
                _directions = std::move(other._directions);
                _absolute_directions = std::move(other._absolute_directions);
                _is_connected = std::move(other._is_connected);
            }
            return *this;
        }

        constexpr ~route_data() = default;

        [[nodiscard]] constexpr bool has_length() const {
            return valid(_length);
        }

        [[nodiscard]] constexpr length_type length() const {
            return _length;
        }

        [[nodiscard]] constexpr bool has_azimuth() const {
            return valid(_azimuth);
        }

        [[nodiscard]] constexpr angle_type azimuth() const {
            return _azimuth;
        }

        [[nodiscard]] constexpr bool has_directions() const {
            return valid(_directions) and valid(_absolute_directions);
        }

        [[nodiscard]] constexpr angle_type directions() const {
            return _directions;
        }

        [[nodiscard]] constexpr angle_type absolute_directions() const {
            return _absolute_directions;
        }

        [[nodiscard]] constexpr bool is_connected() const {
            return _is_connected;
        }

    protected:
        mutable length_type _length;
        mutable angle_type _azimuth;
        mutable angle_type _directions;
        mutable angle_type _absolute_directions;
        mutable bool _is_connected;

        constexpr route_data()
            : _length{not_initialized<length_type>}, _azimuth{not_initialized<angle_type>},
            _directions{not_initialized<length_type>}, _absolute_directions{not_initialized<angle_type>},
            _is_connected{false} {}

        constexpr route_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, const bool is_connected)
            : _length{std::move(length)}, _azimuth{std::move(azimuth)}, _directions{std::move(directions)},
            _absolute_directions{std::move(absolute_directions)}, _is_connected{is_connected} {}

        void _invalidate(const bool length = true, const bool azimuth = true, const bool directions = true,
                const bool connected = true) const {
            if (length) {
                _length = not_initialized<length_type>;
            }
            if (azimuth) {
                _azimuth = not_initialized<angle_type>;
            }
            if (directions) {
                _directions = not_initialized<length_type>;
                _absolute_directions = not_initialized<angle_type>;
            }
            if (connected) {
                _is_connected = false;
            }
        }

        template<is_route Route>
        void _compute_length(const Route &route) const {
            using route_type = Route;

            if (not initialized(_length)) {
                _length = not_valid<length_type>;

                if (route.rich_lines().empty()) {
                    return;
                } else if (route.rich_lines().size() == 1) {
                    const auto &rich_line = route.rich_lines().front();
                    if (route_type::_has_length(rich_line)) {
                        _length = route_type::_get_length(rich_line);
                    }
                } else {
                    bool has_length = false;
                    length_type length = default_float_type<length_type>::v0;

                    for (const auto &rich_line : route.rich_lines()) {
                        if (route_type::_has_length(rich_line)) {
                            length += route_type::_get_length(rich_line);
                            has_length = true;
                        }
                    }

                    if (has_length) {
                        _length = length;
                    }
                }
            }
        }

        template<is_route Route>
        void _compute_azimuth(const Route &route) const {
            using route_type = Route;

            if (not initialized(_azimuth)) {
                _azimuth = not_valid<angle_type>;

                if (route.rich_lines().empty()) {
                    return;
                } else if (route.rich_lines().size() == 1) {
                    const auto &rich_line = route.rich_lines().front();
                    if (route_type::_has_azimuth(rich_line)) {
                        _azimuth = route_type::_get_azimuth(rich_line);
                    }
                } else {
                    for (auto first_it = route.rich_lines().cbegin();
                         first_it != route.rich_lines().cend(); ++first_it) {
                        const auto &first_rich_line = *first_it;

                        if (not route_type::_empty(first_rich_line)) {
                            for (auto last_it = route.rich_lines().crbegin();
                                 last_it != route.rich_lines().crend(); ++last_it) {
                                const auto &last_rich_line = *last_it;

                                if (not route_type::_empty(last_rich_line)) {
                                    const auto &first_point = route_type::_front(first_rich_line);
                                    const auto &last_point = route_type::_back(last_rich_line);

                                    if (not geometry::equals_points(first_point, last_point)) {
                                        _azimuth = geometry::azimuth_deg(first_point, last_point);
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
            }
        }

        template<is_route Route>
        void _compute_directions(const Route &route) const {
            using route_type = Route;

            if (not initialized(_directions) or not initialized(_absolute_directions)) {
                _directions = not_valid<angle_type>;
                _absolute_directions = not_valid<angle_type>;
                _is_connected = true;

                if (route.rich_lines().empty()) {
                    return;
                } else if (route.rich_lines().size() == 1) {
                    const auto &rich_line = route.rich_lines().front();
                    if (route_type::_has_directions(rich_line)) {
                        _directions = route_type::_get_directions(rich_line);
                        _absolute_directions = route_type::_get_absolute_directions(rich_line);
                    }
                } else {
                    bool has_directions = false;
                    angle_type directions = default_float_type<angle_type>::v0;
                    angle_type absolute_directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0; i < route.rich_lines().size() - 1; ++i) {
                        const auto &curr_rich_line = route.rich_lines()[i];
                        const auto &next_rich_line = route.rich_lines()[i + 1];

                        if (not route_type::_empty(curr_rich_line) and not route_type::_empty(next_rich_line)) {
                            // check if rich lines are attached
                            if (route_type::_attaches(curr_rich_line, next_rich_line)) {
                                // add direction inbetween only when attached
                                const auto _curr_size = route_type::_size(curr_rich_line);
                                if (_curr_size >= 2 and
                                    route_type::_has_azimuth(curr_rich_line, _curr_size - 2) and
                                    route_type::_has_azimuth(next_rich_line, 0)) {
                                    const auto direction = geometry::direction_deg(
                                            route_type::_get_azimuth(curr_rich_line, _curr_size - 2),
                                            route_type::_get_azimuth(next_rich_line, 0));
                                    directions += direction;
                                    absolute_directions += std::fabs(direction);
                                    has_directions = true;
                                }
                            } else {
                                _is_connected = false;
                            }
                        }

                        // add remaining directions
                        if (route_type::_has_directions(curr_rich_line)) {
                            directions += route_type::_get_directions(curr_rich_line);
                            absolute_directions += route_type::_get_absolute_directions(curr_rich_line);
                            has_directions = true;
                        }
                        // in last loop, also add next directions
                        if (i + 2 >= route.rich_lines().size() and route_type::_has_directions(next_rich_line)) {
                            directions += route_type::_get_directions(next_rich_line);
                            absolute_directions += route_type::_get_absolute_directions(next_rich_line);
                            has_directions = true;
                        }
                    }

                    if (has_directions) {
                        _directions = directions;
                        _absolute_directions = absolute_directions;
                    }
                }
            }
        }

        void _compute(const is_route auto &route) const {
            _compute_length(route);
            _compute_azimuth(route);
            _compute_directions(route);
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_ROUTE_DATA_HPP
