// Copyright (C) 2021-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_SUBSTRING_HPP
#define MAP_MATCHING_2_SUBSTRING_HPP

#include "rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine>
    RichLine
    substring(const RichLine &line,
              typename RichLine::length_type from_distance, typename RichLine::length_type to_distance,
              const typename RichLine::point_type &from_point = {}, const typename RichLine::point_type &to_point = {},
              bool use_from_point = false, bool use_to_point = false);

    template<typename RichLine>
    RichLine substring_from(const RichLine &line, typename RichLine::length_type from_distance,
                            const typename RichLine::point_type &from_point = {}, bool use_from_point = false);

    template<typename RichLine>
    RichLine substring_to(const RichLine &line, typename RichLine::length_type to_distance,
                          const typename RichLine::point_type &to_point = {}, bool use_to_point = false);

    template<typename RichLine>
    std::pair<typename RichLine::length_type, typename RichLine::length_type>
    distances(const RichLine &line,
              const typename RichLine::point_type &from, const typename RichLine::point_type &to,
              bool calculate_from = true, bool calculate_to = true,
              bool use_from_point = false, bool use_to_point = false);

    template<typename RichLine>
    typename RichLine::length_type
    distance_from(const RichLine &line, const typename RichLine::point_type &from, bool use_from_point = false);

    template<typename RichLine>
    typename RichLine::length_type
    distance_to(const RichLine &line, const typename RichLine::point_type &to, bool use_to_point = false);

    template<typename RichLine>
    RichLine substring_points(const RichLine &line, const typename RichLine::point_type &from,
                              const typename RichLine::point_type &to,
                              bool use_from_point = false, bool use_to_point = false);

    template<typename RichLine>
    RichLine substring_point_from(const RichLine &line, const typename RichLine::point_type &from,
                                  bool use_from_point = false);

    template<typename RichLine>
    RichLine substring_point_to(const RichLine &line, const typename RichLine::point_type &to,
                                bool use_to_point = false);

}

#endif //MAP_MATCHING_2_SUBSTRING_HPP
