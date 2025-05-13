// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_HPP

#include <vector>
#include <list>
#include <set>
#include <memory_resource>

#include "types/geometry/index/compare.hpp"

#include "geometry/rich_type/traits/multi_rich_line.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/direction.hpp"
#include "geometry/algorithm/convert.hpp"
#include "geometry/algorithm/buffer.hpp"

#include "compare/comparison.hpp"

#include "simplify.hpp"

namespace map_matching_2::geometry {

    template<typename MultiRichLine> requires
        (is_multi_rich_line<MultiRichLine>)
    class line_comparator {

    public:
        using multi_rich_line_type = MultiRichLine;
        using rich_line_type = typename multi_rich_line_traits<multi_rich_line_type>::rich_line_type;
        using point_type = typename multi_rich_line_traits<multi_rich_line_type>::point_type;
        using line_type = typename multi_rich_line_traits<multi_rich_line_type>::line_type;
        using segment_type = typename multi_rich_line_traits<multi_rich_line_type>::segment_type;
        using rich_segment_type = typename multi_rich_line_traits<multi_rich_line_type>::rich_segment_type;
        using length_type = typename multi_rich_line_traits<multi_rich_line_type>::length_type;
        using angle_type = typename multi_rich_line_traits<multi_rich_line_type>::angle_type;
        using distance_type = typename multi_rich_line_traits<multi_rich_line_type>::distance_type;
        using result_type = comparison<rich_segment_type>;

        bool debug = false;

        double simplifying_tolerance = 0.1;
        double simplifying_reverse_tolerance = 0.1;
        double adoption_distance_tolerance = 0.1;
        double split_distance_tolerance = 1.0;
        double split_direction_tolerance = 90.0;

        constexpr line_comparator() = default;

        constexpr line_comparator(const double simplifying_tolerance, const double simplifying_reverse_tolerance,
                const double adoption_distance_tolerance, const double split_distance_tolerance,
                const double split_direction_tolerance, const bool debug)
            : debug{debug},
            simplifying_tolerance{simplifying_tolerance},
            simplifying_reverse_tolerance{simplifying_reverse_tolerance},
            adoption_distance_tolerance{adoption_distance_tolerance},
            split_distance_tolerance{split_distance_tolerance},
            split_direction_tolerance{split_direction_tolerance} {}

        [[nodiscard]] result_type compare(
                const multi_rich_line_type &ground_truth, const multi_rich_line_type &compare) const {
            std::pmr::list<rich_segment_type> a_segments_list{&_buffer};
            for (const rich_line_type &a : ground_truth.rich_lines()) {
                a_segments_list.splice(a_segments_list.end(),
                        line2rich_segments<std::pmr::list<rich_segment_type>>(a.line(), &_buffer));
            }
            std::pmr::list<rich_segment_type> b_segments_list{&_buffer};
            for (const rich_line_type &b : compare.rich_lines()) {
                b_segments_list.splice(b_segments_list.end(),
                        line2rich_segments<std::pmr::list<rich_segment_type>>(b.line(), &_buffer));
            }

            auto _comparison = this->_compare(
                    std::move(a_segments_list), std::move(b_segments_list),
                    ground_truth.has_length(),
                    ground_truth.has_length()
                    ? ground_truth.length()
                    : default_float_type<length_type>::v0,
                    compare.has_length(),
                    compare.has_length()
                    ? compare.length()
                    : default_float_type<length_type>::v0);

            return _comparison;
        }

        [[nodiscard]] result_type compare(const rich_line_type &ground_truth, const rich_line_type &compare) const {
            std::pmr::list<rich_segment_type> a_segments_list =
                    line2rich_segments<std::pmr::list<rich_segment_type>>(ground_truth.line(), &_buffer);

            std::pmr::list<rich_segment_type> b_segments_list =
                    line2rich_segments<std::pmr::list<rich_segment_type>>(compare.line(), &_buffer);

            auto _comparison = this->_compare(
                    std::move(a_segments_list), std::move(b_segments_list),
                    ground_truth.has_length(),
                    ground_truth.has_length()
                    ? ground_truth.length()
                    : default_float_type<length_type>::v0,
                    compare.has_length(),
                    compare.has_length()
                    ? compare.length()
                    : default_float_type<length_type>::v0);

            return _comparison;
        }

    private:
        using points_set_type = std::pmr::set<point_type>;

        using rtree_points_type = compare_rtree_points_type<point_type>;
        using rtree_segments_type = compare_rtree_segments_type<rich_segment_type>;
        using rtree_segments_key_type = typename rtree_segments_type::value_type;

        mutable std::pmr::monotonic_buffer_resource _buffer{1024};

        [[nodiscard]] result_type _compare(
                std::pmr::list<rich_segment_type> a_segments_list, std::pmr::list<rich_segment_type> b_segments_list,
                bool ground_truth_has_length, length_type ground_truth_length,
                bool compare_has_length, length_type compare_length) const {
            BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

            std::pmr::list<rich_segment_type> corrects{&_buffer}, error_adds{&_buffer}, error_misses{&_buffer};

            // if one of both lines is empty, easy return
            if ((a_segments_list.empty() and b_segments_list.empty())) {
                return result_type{
                        default_float_type<length_type>::v1,
                        default_float_type<length_type>::v0,
                        default_float_type<length_type>::v0,
                        default_float_type<length_type>::v0,
                        default_float_type<length_type>::v0,
                        std::move(corrects), std::move(error_adds),
                        std::move(error_misses)
                };
            } else if (a_segments_list.empty() and not b_segments_list.empty()) {
                return result_type{
                        default_float_type<length_type>::v0,
                        std::numeric_limits<length_type>::infinity(),
                        default_float_type<length_type>::v0,
                        compare_has_length
                        ? compare_length
                        : default_float_type<length_type>::v0,
                        default_float_type<length_type>::v0,
                        std::move(corrects), std::move(b_segments_list),
                        std::move(error_misses)
                };
            } else if (not a_segments_list.empty() and b_segments_list.empty()) {
                return result_type{
                        default_float_type<length_type>::v0,
                        default_float_type<length_type>::v1,
                        default_float_type<length_type>::v0,
                        default_float_type<length_type>::v0,
                        ground_truth_has_length
                        ? ground_truth_length
                        : default_float_type<length_type>::v0,
                        std::move(corrects), std::move(error_adds),
                        std::move(a_segments_list)
                };
            }

            if (debug) {
                std::cout << "Ground Truth Segments:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // simplify with Douglas-Peucker algorithm
            simplify_rich_segments(a_segments_list, true, simplifying_tolerance, simplifying_reverse_tolerance);
            simplify_rich_segments(b_segments_list, true, simplifying_tolerance, simplifying_reverse_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Simplified:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Simplified:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // adapt points for erasing tiny differences
            _adapt_points(a_segments_list, a_segments_list, adoption_distance_tolerance);
            _adapt_points(b_segments_list, b_segments_list, adoption_distance_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Ground Truth Adapted:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Compare Adapted:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // adapt points again to each other for erasing more tiny differences
            _adapt_points(a_segments_list, b_segments_list, adoption_distance_tolerance);
            _adapt_points(b_segments_list, a_segments_list, adoption_distance_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Compare Adapted:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Ground Truth Adapted:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // fix split points, first self for reverse and loop parts overlapping
            _split_segments(a_segments_list, a_segments_list, split_distance_tolerance, split_direction_tolerance);
            _split_segments(b_segments_list, b_segments_list, split_distance_tolerance, split_direction_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Ground Truth Split:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Compare Split:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // then split with each other for finding overlaps of both lines
            _split_segments(a_segments_list, b_segments_list, split_distance_tolerance, split_direction_tolerance);
            _split_segments(b_segments_list, a_segments_list, split_distance_tolerance, split_direction_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Compare Split:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Ground Truth Split:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // adapt points for erasing tiny differences
            _adapt_points(a_segments_list, a_segments_list, adoption_distance_tolerance);
            _adapt_points(b_segments_list, b_segments_list, adoption_distance_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Ground Truth Again Adapted:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Compare Again Adapted:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // adapt points again to each other for erasing more tiny differences
            _adapt_points(a_segments_list, b_segments_list, adoption_distance_tolerance);
            _adapt_points(b_segments_list, a_segments_list, adoption_distance_tolerance);

            if (debug) {
                std::cout << "Ground Truth Segments Compare Again Adapted:\n";
                for (const auto &a_segment : a_segments_list) {
                    std::cout << a_segment.wkt() << "\n";
                }
                std::cout << "Compare Segments Ground Truth Again Adapted:\n";
                for (const auto &b_segment : b_segments_list) {
                    std::cout << b_segment.wkt() << "\n";
                }
            }

            // convert to vector for further processing
            std::vector<rich_segment_type> a_segments{a_segments_list.cbegin(), a_segments_list.cend()};
            std::vector<rich_segment_type> b_segments{b_segments_list.cbegin(), b_segments_list.cend()};

            // temporary collections
            std::pmr::list<rich_segment_type> reverse_a_segments{&_buffer}, reverse_b_segments{&_buffer};

            std::size_t i = 0, j = 0;
            while (i < a_segments.size() and j < b_segments.size()) {
                if (geometry::equals_points(a_segments[i].segment().first, b_segments[j].segment().first) and
                    geometry::equals_points(a_segments[i].segment().second, b_segments[j].segment().second)) {
                    // equal parts, no error
                    corrects.emplace_back(a_segments[i]);
                    i++;
                    j++;
                } else {
                    const auto a_reverse = _detect_reverse(a_segments, i, a_segments.size());
                    const auto b_reverse = _detect_reverse(b_segments, j, b_segments.size());

                    // treat reverse movements
                    const auto is_a_reverse = std::get<0>(a_reverse);
                    if (is_a_reverse) {
                        const auto a_reverse_start = std::get<1>(a_reverse);
                        const auto a_reverse_end = std::get<2>(a_reverse);
                        for (std::size_t r = a_reverse_start; r < a_reverse_end; ++r) {
                            reverse_a_segments.emplace_back(a_segments[r]);
                        }
                        i = a_reverse_end;
                    }
                    const auto is_b_reverse = std::get<0>(b_reverse);
                    if (is_b_reverse) {
                        const auto b_reverse_start = std::get<1>(b_reverse);
                        const auto b_reverse_end = std::get<2>(b_reverse);
                        for (std::size_t r = b_reverse_start; r < b_reverse_end; ++r) {
                            reverse_b_segments.emplace_back(b_segments[r]);
                        }
                        j = b_reverse_end;
                    }

                    if (i >= a_segments.size() or j >= b_segments.size()) {
                        break;
                    }

                    if (not is_a_reverse and not is_b_reverse) {
                        // compare point is somewhere outside of ground truth
                        const auto merge = _find_merge(a_segments, b_segments, i, a_segments.size(), j,
                                b_segments.size());

                        // treat errors
                        const auto i_merge = std::get<0>(merge);
                        const auto j_merge = std::get<1>(merge);
                        for (std::size_t i_c = i; i_c < i_merge; ++i_c) {
                            error_misses.emplace_back(a_segments[i_c]);
                        }
                        for (std::size_t j_c = j; j_c < j_merge; ++j_c) {
                            error_adds.emplace_back(b_segments[j_c]);
                        }
                        i = i_merge;
                        j = j_merge;
                    }
                }
            }

            if (i < a_segments.size()) {
                for (std::size_t i_c = i; i_c < a_segments.size(); ++i_c) {
                    error_misses.emplace_back(a_segments[i_c]);
                }
            }
            if (j < b_segments.size()) {
                for (std::size_t j_c = j; j_c < b_segments.size(); ++j_c) {
                    error_adds.emplace_back(b_segments[j_c]);
                }
            }

            // reverses that are part of both lines were correct
            bool restart;
            auto reverse_b_it = reverse_b_segments.begin();
            while (reverse_b_it != reverse_b_segments.end()) {
                restart = false;
                auto reverse_a_it = reverse_a_segments.begin();
                while (reverse_a_it != reverse_a_segments.end()) {
                    if (geometry::equals_points((*reverse_a_it).segment().first, (*reverse_b_it).segment().first) and
                        geometry::equals_points((*reverse_a_it).segment().second, (*reverse_b_it).segment().second)) {
                        corrects.emplace_back(*reverse_a_it);
                        reverse_a_segments.erase(reverse_a_it);
                        reverse_b_segments.erase(reverse_b_it);
                        reverse_b_it = reverse_b_segments.begin();
                        restart = true;
                        break;
                    }
                    ++reverse_a_it;
                }
                if (not restart) {
                    ++reverse_b_it;
                }
            }

            // supposed added parts that were part of the ground truth reverse line were in fact correct
            auto error_adds_it = error_adds.begin();
            while (error_adds_it != error_adds.end()) {
                restart = false;
                auto reverse_a_it = reverse_a_segments.begin();
                while (reverse_a_it != reverse_a_segments.end()) {
                    if (geometry::equals_points((*reverse_a_it).segment().first, (*error_adds_it).segment().first) and
                        geometry::equals_points((*reverse_a_it).segment().second, (*error_adds_it).segment().second)) {
                        corrects.emplace_back(*reverse_a_it);
                        reverse_a_segments.erase(reverse_a_it);
                        error_adds.erase(error_adds_it);
                        error_adds_it = error_adds.begin();
                        restart = true;
                        break;
                    }
                    ++reverse_a_it;
                }
                if (not restart) {
                    ++error_adds_it;
                }
            }

            // supposed missed parts that were part of the compare reverse line were in fact correct
            auto error_misses_it = error_misses.begin();
            while (error_misses_it != error_misses.end()) {
                restart = false;
                reverse_b_it = reverse_b_segments.begin();
                while (reverse_b_it != reverse_b_segments.end()) {
                    if (geometry::equals_points((*reverse_b_it).segment().first, (*error_misses_it).segment().first) and
                        geometry::equals_points((*reverse_b_it).segment().second,
                                (*error_misses_it).segment().second)) {
                        corrects.emplace_back(*reverse_b_it);
                        reverse_b_segments.erase(reverse_b_it);
                        error_misses.erase(error_misses_it);
                        error_misses_it = error_misses.begin();
                        restart = true;
                        break;
                    }
                    ++reverse_b_it;
                }
                if (not restart) {
                    ++error_misses_it;
                }
            }

            // remaining duplicate lines are missed lines because already matched duplicate lines were removed above
            for (auto reverse_a_it = reverse_a_segments.begin();
                 reverse_a_it != reverse_a_segments.end(); ++reverse_a_it) {
                if (std::find_if(error_misses.begin(), error_misses.end(), [&](const auto &error_miss) {
                    return geometry::equals_points((*reverse_a_it).segment().first, error_miss.segment().first) and
                            geometry::equals_points((*reverse_a_it).segment().second, error_miss.segment().second);
                }) == error_misses.end()) {
                    error_misses.emplace_back(*reverse_a_it);
                }
            }

            // remaining reverse lines were added erroneously
            for (reverse_b_it = reverse_b_segments.begin();
                 reverse_b_it != reverse_b_segments.end(); ++reverse_b_it) {
                if (std::find_if(error_adds.begin(), error_adds.end(), [&](const auto &error_add) {
                    return geometry::equals_points((*reverse_b_it).segment().first, error_add.segment().first) and
                            geometry::equals_points((*reverse_b_it).segment().second, error_add.segment().second);
                }) == error_adds.end()) {
                    error_adds.emplace_back(*reverse_b_it);
                }
            }

            // missed lines that we added were correctly found, can happen due to early merging within loops
            error_misses_it = error_misses.begin();
            while (error_misses_it != error_misses.end()) {
                restart = false;
                error_adds_it = error_adds.begin();
                while (error_adds_it != error_adds.end()) {
                    if (geometry::equals_points((*error_adds_it).segment().first,
                                (*error_misses_it).segment().first) and
                        geometry::equals_points((*error_adds_it).segment().second,
                                (*error_misses_it).segment().second)) {
                        corrects.emplace_back(*error_adds_it);
                        error_adds.erase(error_adds_it);
                        error_misses.erase(error_misses_it);
                        error_misses_it = error_misses.begin();
                        restart = true;
                        break;
                    }
                    ++error_adds_it;
                }
                if (not restart) {
                    ++error_misses_it;
                }
            }

            // sum lengths
            length_type correct = default_float_type<length_type>::v0;
            auto corrects_it = corrects.begin();
            while (corrects_it != corrects.end()) {
                const auto &correct_segment = *corrects_it;
                length_type length = geometry::default_float_type<length_type>::v0;
                if (correct_segment.has_length()) {
                    length = correct_segment.length();
                }
                ++corrects_it;
                if (length == geometry::default_float_type<length_type>::v0) {
                    corrects.erase(std::prev(corrects_it));
                } else {
                    correct += length;
                }
            }

            length_type error_added = default_float_type<length_type>::v0;
            error_adds_it = error_adds.begin();
            while (error_adds_it != error_adds.end()) {
                const auto &error_add_segment = *error_adds_it;
                length_type length = geometry::default_float_type<length_type>::v0;
                if (error_add_segment.has_length()) {
                    length = error_add_segment.length();
                }
                ++error_adds_it;
                if (length == geometry::default_float_type<length_type>::v0) {
                    error_adds.erase(std::prev(error_adds_it));
                } else {
                    error_added += length;
                }
            }

            length_type error_missed = default_float_type<length_type>::v0;
            error_misses_it = error_misses.begin();
            while (error_misses_it != error_misses.end()) {
                const auto &error_miss_segment = *error_misses_it;
                length_type length = geometry::default_float_type<length_type>::v0;
                if (error_miss_segment.has_length()) {
                    length = error_miss_segment.length();
                }
                ++error_misses_it;
                if (length == geometry::default_float_type<length_type>::v0) {
                    error_misses.erase(std::prev(error_misses_it));
                } else {
                    error_missed += length;
                }
            }

            length_type error_fraction = default_float_type<length_type>::v0;
            if (error_added != default_float_type<length_type>::v0 or
                error_missed != default_float_type<length_type>::v0) {
                // in case errors have happened, compute error fraction
                if (ground_truth_has_length and ground_truth_length > default_float_type<length_type>::v0) {
                    error_fraction = (error_added + error_missed) / ground_truth_length;
                } else {
                    // when ground truth has no length or a length of zero, the equation is undefined,
                    // error is infinity by definition
                    error_fraction = std::numeric_limits<length_type>::infinity();
                }
            }

            length_type correct_fraction = default_float_type<length_type>::v0;
            if (error_fraction == default_float_type<length_type>::v0) {
                // when no error has happened, correct fraction is per definition 1.0
                correct_fraction = default_float_type<length_type>::v1;
            } else {
                // when errors have happened, compute correct fraction
                if (ground_truth_has_length and ground_truth_length > default_float_type<length_type>::v0 and
                    compare_has_length and compare_length > default_float_type<length_type>::v0) {
                    // the accuracy of a match depends on whether the match is shorter or longer than the ground truth
                    // when the match is shorter we must have missed parts, so our accuracy depends on the correctly found
                    // parts from the ground truth; in the other case we added unnecessary parts, so our accuracy depends
                    // on how much correct parts we still found in our match
                    correct_fraction = correct / std::max(ground_truth_length, compare_length);
                } else if (
                    (not ground_truth_has_length or
                        ground_truth_length == default_float_type<length_type>::v0) and
                    (not compare_has_length or compare_length == default_float_type<length_type>::v0)) {
                    // when both lengths are zero and the unlikely event happened that this was not caught
                    // in the beginning of this method, we are actually completely correct
                    correct_fraction = default_float_type<length_type>::v1;
                } else {
                    // when one of both lines has no length or a length of zero, the equation is undefined,
                    // accuracy is zero because either the empty ground truth was not found or the match was empty itself
                    correct_fraction = default_float_type<length_type>::v0;
                }

                // in case of rounding errors around 1.0 or 0.0 cut to valid range
                correct_fraction = std::min(default_float_type<length_type>::v1,
                        std::max(default_float_type<length_type>::v0, correct_fraction));
            }

            return result_type{
                    correct_fraction, error_fraction, correct, error_added, error_missed,
                    std::move(corrects), std::move(error_adds), std::move(error_misses)
            };
        }

        [[nodiscard]] points_set_type _create_points_set(const std::pmr::list<rich_segment_type> &segments) const {
            points_set_type points_set{&_buffer};
            for (const auto &segment : segments) {
                points_set.emplace(segment.first());
                points_set.emplace(segment.second());
            }
            return points_set;
        }

        [[nodiscard]] rtree_points_type _create_points_rtree(const std::pmr::list<rich_segment_type> &segments) const {
            points_set_type points_set = _create_points_set(segments);
            rtree_points_type points_index{points_set};
            return points_index;
        }

        [[nodiscard]] rtree_segments_type _create_segments_rtree(std::pmr::list<rich_segment_type> &segments) const {
            std::vector<rtree_segments_key_type> segments_list;
            segments_list.reserve(segments.size());
            for (auto it = segments.begin(); it != segments.end(); ++it) {
                segments_list.emplace_back(rtree_segments_key_type{(*it).segment(), it});
            }
            rtree_segments_type segments_index{segments_list};
            return segments_index;
        }

        void _adapt_points(std::pmr::list<rich_segment_type> &segments,
                const std::pmr::list<rich_segment_type> &adopt_segments,
                double distance_tolerance) const {
            rtree_points_type points_index = _create_points_rtree(adopt_segments);

            const auto buffer_radius = std::max(1e-3, 2 * distance_tolerance);

            auto adapt_point = [&](auto &segment_it, const bool first) {
                const segment_type &segment = segment_it->segment();
                const point_type &point = first ? segment.first : segment.second;

                const auto buffer = geometry::buffer_box(point, buffer_radius);
                std::pmr::list<point_type> result_points{&_buffer};
                points_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(result_points));

                distance_type nearest_distance = std::numeric_limits<distance_type>::infinity();
                const point_type *nearest_point = nullptr;

                for (const auto &result_point : result_points) {
                    if (not geometry::equals_points(point, result_point)) {
                        const auto distance = geometry::distance(point, result_point);
                        if (distance < distance_tolerance and distance < nearest_distance) {
                            nearest_distance = distance;
                            nearest_point = &result_point;
                        }
                    }
                }

                if (nearest_point != nullptr) {
                    *segment_it = first
                                  ? rich_segment_type{segment_type{*nearest_point, segment.second}}
                                  : rich_segment_type{segment_type{segment.first, *nearest_point}};
                    if (not(segment_it == segments.begin() and
                            geometry::equals_points(point, segment_it->segment().first)) and
                        std::next(segment_it) != segments.end()) {
                        auto &next_segment = *std::next(segment_it);
                        if (not geometry::equals_points(next_segment.segment().first, point)) {
                            next_segment = rich_segment_type{segment_type{point, next_segment.segment().second}};
                        }
                    }
                }
            };

            for (auto segment_it = segments.begin(); segment_it != segments.end(); ++segment_it) {
                if (segment_it == segments.begin()) {
                    adapt_point(segment_it, true);
                }
                adapt_point(segment_it, false);
            }

            // erase empty strings after adoption
            auto segment_it = segments.begin();
            while (segment_it != segments.end()) {
                auto &segment = *segment_it;
                ++segment_it;
                if (not segment.has_length() or segment.length() == geometry::default_float_type<length_type>::v0) {
                    segments.erase(std::prev(segment_it));
                }
            }
        }

        void _split_segments(std::pmr::list<rich_segment_type> &segments,
                const std::pmr::list<rich_segment_type> &compare_segments,
                double distance_tolerance, double direction_tolerance) const {
            points_set_type points_set = _create_points_set(compare_segments);
            rtree_segments_type segments_index = _create_segments_rtree(segments);

            const auto buffer_radius = std::max(1e-3, 2 * distance_tolerance);

            for (const auto &point : points_set) {
                const auto buffer = geometry::buffer_box<point_type>(point, buffer_radius);
                std::vector<rtree_segments_key_type> result_segments;
                segments_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(result_segments));

                auto result_it = result_segments.begin();
                while (result_it != result_segments.end()) {
                    ++result_it;
                    auto &result = *std::prev(result_it);

                    const segment_type &segment = result.first;

                    if (not geometry::equals_points(segment.first, point) and
                        not geometry::equals_points(segment.second, point)) {
                        const auto distance = geometry::distance(segment, point);

                        if (distance < distance_tolerance) {
                            rich_segment_type from{segment_type{segment.first, point}};
                            rich_segment_type to{segment_type{point, segment.second}};

                            if (from.has_length() and to.has_length() and from.has_azimuth() and to.has_azimuth()) {
                                const auto direction = std::fabs(geometry::direction_deg(from.azimuth(), to.azimuth()));
                                if (direction < direction_tolerance) {
                                    auto segment_it = result.second;
                                    auto new_segment_it = segments.insert(std::next(segment_it), std::move(to));
                                    *segment_it = std::move(from);

                                    segments_index.insert(rtree_segments_key_type{(*segment_it).segment(), segment_it});
                                    segments_index.remove(result);
                                    segments_index.insert(
                                            rtree_segments_key_type{(*new_segment_it).segment(), new_segment_it});
                                }
                            }
                        }
                    }
                }
            }
        }

        [[nodiscard]] std::tuple<bool, std::size_t, std::size_t>
        _detect_reverse(const std::vector<rich_segment_type> &segments, std::size_t start, std::size_t end) const {
            bool start_found = false, end_found = false;

            if (start >= 1 and
                geometry::equals_points(segments[start - 1].segment().second, segments[start].segment().first) and
                geometry::equals_points(segments[start - 1].segment().first, segments[start].segment().second)) {
                // segments are equal in reverse, reverse detected

                const auto &start_point = segments[start].segment().first;
                start_found = true;

                for (std::size_t i = start + 1; i < end; ++i) {
                    if (geometry::equals_points(segments[i].segment().first, start_point)) {
                        // end point detected
                        end = i;
                        end_found = true;
                    }
                }
            }

            return std::tuple{start_found and end_found, start, end};
        }

        [[nodiscard]] std::tuple<std::size_t, std::size_t>
        _find_merge(const std::vector<rich_segment_type> &a_segments, const std::vector<rich_segment_type> &b_segments,
                std::size_t a_start, std::size_t a_end, std::size_t b_start, std::size_t b_end) const {
            if (geometry::equals_points(a_segments[a_start].segment().first, b_segments[b_start].segment().first) and
                not geometry::equals_points(a_segments[a_start].segment().second,
                        b_segments[b_start].segment().second)) {
                b_start++;
            }

            for (std::size_t j = b_start; j < b_end; ++j) {
                const auto &b_point = b_segments[j].segment().first;
                for (std::size_t i = a_start; i < a_end; ++i) {
                    const auto &a_point = a_segments[i].segment().first;
                    if (geometry::equals_points(a_point, b_point)) {
                        return std::tuple{i, j};
                    }
                }
            }

            return std::tuple{a_segments.size(), b_segments.size()};
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_COMPARE_HPP
