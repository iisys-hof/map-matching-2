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

#ifndef MAP_MATCHING_2_COMPARE_HPP
#define MAP_MATCHING_2_COMPARE_HPP

#include "rich_line.hpp"
#include "multi_rich_line.hpp"

#include "util.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine>
    struct comparison {
        using rich_line_type = RichLine;
        using segment_type = typename RichLine::segment_type;
        using rich_segment_type = typename RichLine::rich_segment_type;
        using length_type = typename RichLine::length_type;

        length_type correct_fraction;
        length_type error_fraction;
        length_type correct;
        length_type error_added;
        length_type error_missed;
        std::list<rich_segment_type> corrects;
        std::list<rich_segment_type> error_adds;
        std::list<rich_segment_type> error_misses;

    };

    template<typename RichLine>
    class line_comparator {

    public:
        using point_type = typename RichLine::point_type;
        using line_type = typename RichLine::line_type;
        using segment_type = typename RichLine::segment_type;
        using rich_segment_type = typename RichLine::rich_segment_type;
        using length_type = typename RichLine::length_type;
        using angle_type = typename RichLine::angle_type;
        using distance_type = typename boost::geometry::default_distance_result<point_type, point_type>::type;
        using rich_line_type = RichLine;
        using multi_rich_line_type = multi_rich_line<line_type>;

        bool debug = false;

        double simplifying_tolerance = 0.1;
        double simplifying_reverse_tolerance = 0.1;
        double adoption_distance_tolerance = 0.1;
        double split_distance_tolerance = 1.0;
        double split_direction_tolerance = 90.0;

        comparison<RichLine>
        compare(const multi_rich_line_type &ground_truth, const multi_rich_line_type &compare) const;

        comparison<RichLine>
        compare(const RichLine &ground_truth, const RichLine &compare) const;

    private:
        using points_set_type = std::set<point_type, decltype(&geometry::point_set_comparator<point_type>)>;
        using rtree_segments_key_type = std::pair<segment_type, typename std::list<rich_segment_type>::iterator>;

        using rtree_points_type = boost::geometry::index::rtree<point_type, boost::geometry::index::rstar<16>>;
        using rtree_segments_type = boost::geometry::index::rtree<rtree_segments_key_type, boost::geometry::index::rstar<16>>;

        [[nodiscard]] points_set_type _create_points_set(const std::list<rich_segment_type> &segments) const;

        [[nodiscard]] rtree_points_type _create_points_rtree(const std::list<rich_segment_type> &segments) const;

        [[nodiscard]] rtree_segments_type _create_segments_rtree(std::list<rich_segment_type> &segments) const;

        void
        _adapt_points(std::list<rich_segment_type> &segments, const std::list<rich_segment_type> &adopt_segments,
                      double distance_tolerance) const;

        void
        _split_segments(std::list<rich_segment_type> &segments, const std::list<rich_segment_type> &compare_segments,
                        double distance_tolerance, double direction_tolerance) const;

        std::tuple<bool, std::size_t, std::size_t>
        _detect_reverse(const std::vector<rich_segment_type> &segments, std::size_t start, std::size_t end) const;

        std::tuple<std::size_t, std::size_t>
        _find_merge(const std::vector<rich_segment_type> &a_segments, const std::vector<rich_segment_type> &b_segments,
                    std::size_t a_start, std::size_t a_end, std::size_t b_start, std::size_t b_end) const;

    };

}

#endif //MAP_MATCHING_2_COMPARE_HPP
