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

#ifndef MAP_MATCHING_2_RICH_TYPE_CONCEPTS_HPP
#define MAP_MATCHING_2_RICH_TYPE_CONCEPTS_HPP

#include <concepts>

#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<typename RichSegment>
    concept is_rich_segment = requires(RichSegment &rich_segment) {
        typename RichSegment::segment_type;
        { rich_segment.segment() } -> std::same_as<typename RichSegment::segment_type &>;
        { rich_segment.has_length() } -> std::same_as<bool>;
        { rich_segment.has_azimuth() } -> std::same_as<bool>;
        { rich_segment.length() } -> std::same_as<typename data<typename RichSegment::segment_type>::length_type>;
        { rich_segment.azimuth() } -> std::same_as<typename data<typename RichSegment::segment_type>::angle_type>;
    };

    template<typename RichLine>
    concept is_rich_line = requires(RichLine &rich_line) {
        typename RichLine::line_type;
        { rich_line.line() } -> std::same_as<typename RichLine::line_type &>;
        { rich_line.has_length() } -> std::same_as<bool>;
        { rich_line.has_azimuth() } -> std::same_as<bool>;
        { rich_line.has_directions() } -> std::same_as<bool>;
        { rich_line.length() } -> std::same_as<typename data<typename RichLine::line_type>::distance_type>;
        { rich_line.azimuth() } -> std::same_as<typename data<typename RichLine::line_type>::angle_type>;
        { rich_line.directions() } -> std::same_as<typename data<typename RichLine::line_type>::angle_type>;
        { rich_line.absolute_directions() } -> std::same_as<typename data<typename RichLine::line_type>::angle_type>;
    };

    template<typename MultiRichLine>
    concept is_multi_rich_line = requires(MultiRichLine &multi_rich_line) {
        typename MultiRichLine::line_type;
        typename MultiRichLine::rich_line_type;
        typename MultiRichLine::rich_lines_container_type;
        requires is_rich_line<typename MultiRichLine::rich_line_type>;
        { multi_rich_line.rich_lines() } -> std::same_as<typename MultiRichLine::rich_lines_container_type &>;
        { multi_rich_line.has_length() } -> std::same_as<bool>;
        { multi_rich_line.has_azimuth() } -> std::same_as<bool>;
        { multi_rich_line.has_directions() } -> std::same_as<bool>;
        { multi_rich_line.length() } -> std::same_as<typename data<typename MultiRichLine::line_type>::length_type>;
        { multi_rich_line.azimuth() } -> std::same_as<typename data<typename MultiRichLine::line_type>::angle_type>;
        { multi_rich_line.directions() } -> std::same_as<typename data<typename MultiRichLine::line_type>::angle_type>;
        {
            multi_rich_line.absolute_directions()
        } -> std::same_as<typename data<typename MultiRichLine::line_type>::angle_type>;
    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_CONCEPTS_HPP
