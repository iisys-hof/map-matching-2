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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_COMMON_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_COMMON_HPP

#include <cstddef>
#include <limits>
#include <format>
#include <iostream>

namespace map_matching_2::geometry::network {

    struct route_return_line_parts {
        struct part {
            struct position {
                std::size_t ref{std::numeric_limits<std::size_t>::max()};
                std::size_t index{std::numeric_limits<std::size_t>::max()};

                constexpr bool operator==(const position &_other) const {
                    if (this != &_other) {
                        return this->ref == _other.ref and this->index == _other.index;
                    }
                    return true;
                }
            };

            position start{}, end{};

            constexpr bool operator==(const part &_other) const {
                if (this != &_other) {
                    return this->start == _other.start and this->end == _other.end;
                }
                return true;
            }
        };

        part self{}, other{};

        constexpr route_return_line_parts() = default;

        constexpr route_return_line_parts(
                const std::size_t self_start_ref, const std::size_t self_start_index,
                const std::size_t self_end_ref, const std::size_t self_end_index,
                const std::size_t other_start_ref, const std::size_t other_start_index,
                const std::size_t other_end_ref, const std::size_t other_end_index)
            : self{
                    part{
                            part::position{self_start_ref, self_start_index},
                            part::position{self_end_ref, self_end_index}
                    }
            },
            other{
                    part{
                            part::position{other_start_ref, other_start_index},
                            part::position{other_end_ref, other_end_index}
                    }
            } {}

        constexpr bool operator==(const route_return_line_parts &_other) const {
            if (this != &_other) {
                return this->self == _other.self and this->other == _other.other;
            }
            return true;
        }

        friend std::ostream &operator<<(std::ostream &out, const route_return_line_parts &parts) {
            return out << std::format(
                    "route_return_line_parts("
                    "self.start.ref: {}, self.start.index: {}, self.end.ref: {}, self.end.index: {},"
                    "other.start.ref: {}, other.start.index: {}, other.end.ref: {}, other.end.index: {})",
                    parts.self.start.ref, parts.self.start.index, parts.self.end.ref, parts.self.end.index,
                    parts.other.start.ref, parts.other.start.index, parts.other.end.ref, parts.other.end.index);
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_COMMON_HPP
