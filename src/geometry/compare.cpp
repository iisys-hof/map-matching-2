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

#include <unordered_set>

#include "compare.hpp"

#include "types.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine>
    comparison<RichLine>
    line_comparator<RichLine>::compare(const multi_rich_line_type &ground_truth,
                                       const multi_rich_line_type &compare) const {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<line_type>));

        std::list<rich_segment_type> corrects, error_adds, error_misses;

        // segments
        std::list<rich_segment_type> a_segments_list;
        for (const rich_line_type &a: ground_truth.rich_lines) {
            a_segments_list.insert(a_segments_list.end(), a.rich_segments.cbegin(), a.rich_segments.cend());
        }
        std::list<rich_segment_type> b_segments_list;
        for (const rich_line_type &b: compare.rich_lines) {
            b_segments_list.insert(b_segments_list.end(), b.rich_segments.cbegin(), b.rich_segments.cend());
        }

        // if one of both lines is empty, easy return
        if ((a_segments_list.empty() and b_segments_list.empty())) {
            return comparison<RichLine>{default_float_type<length_type>::v1, default_float_type<length_type>::v0,
                                        default_float_type<length_type>::v0, default_float_type<length_type>::v0,
                                        default_float_type<length_type>::v0,
                                        std::move(corrects), std::move(error_adds), std::move(error_misses)};
        } else if (a_segments_list.empty() and not b_segments_list.empty()) {
            return comparison<RichLine>{default_float_type<length_type>::v0,
                                        std::numeric_limits<length_type>::infinity(),
                                        default_float_type<length_type>::v0,
                                        compare.has_length ? compare.length : default_float_type<length_type>::v0,
                                        default_float_type<length_type>::v0,
                                        std::move(corrects), std::move(b_segments_list), std::move(error_misses)};
        } else if (not a_segments_list.empty() and b_segments_list.empty()) {
            return comparison<RichLine>{default_float_type<length_type>::v0,
                                        std::numeric_limits<length_type>::infinity(),
                                        default_float_type<length_type>::v0, default_float_type<length_type>::v0,
                                        ground_truth.has_length ? ground_truth.length
                                                                : default_float_type<length_type>::v0,
                                        std::move(corrects), std::move(error_adds), std::move(a_segments_list)};
        }

        if (debug) {
            std::cout << "Ground Truth Segments:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // simplify with Douglas-Peucker algorithm
        rich_segment_type::simplify(a_segments_list, true, simplifying_tolerance, simplifying_reverse_tolerance);
        rich_segment_type::simplify(b_segments_list, true, simplifying_tolerance, simplifying_reverse_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Simplified:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Simplified:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // adapt points for erasing tiny differences
        _adapt_points(a_segments_list, a_segments_list, adoption_distance_tolerance);
        _adapt_points(b_segments_list, b_segments_list, adoption_distance_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Ground Truth Adapted:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Compare Adapted:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // adapt points again to each other for erasing more tiny differences
        _adapt_points(a_segments_list, b_segments_list, adoption_distance_tolerance);
        _adapt_points(b_segments_list, a_segments_list, adoption_distance_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Compare Adapted:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Ground Truth Adapted:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // fix split points, first self for reverse and loop parts overlapping
        _split_segments(a_segments_list, a_segments_list, split_distance_tolerance, split_direction_tolerance);
        _split_segments(b_segments_list, b_segments_list, split_distance_tolerance, split_direction_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Ground Truth Split:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Compare Split:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // then split with each other for finding overlaps of both lines
        _split_segments(a_segments_list, b_segments_list, split_distance_tolerance, split_direction_tolerance);
        _split_segments(b_segments_list, a_segments_list, split_distance_tolerance, split_direction_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Compare Split:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Ground Truth Split:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // adapt points for erasing tiny differences
        _adapt_points(a_segments_list, a_segments_list, adoption_distance_tolerance);
        _adapt_points(b_segments_list, b_segments_list, adoption_distance_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Ground Truth Again Adapted:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Compare Again Adapted:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // adapt points again to each other for erasing more tiny differences
        _adapt_points(a_segments_list, b_segments_list, adoption_distance_tolerance);
        _adapt_points(b_segments_list, a_segments_list, adoption_distance_tolerance);

        if (debug) {
            std::cout << "Ground Truth Segments Compare Again Adapted:\n";
            for (const auto &a_segment: a_segments_list) {
                std::cout << a_segment.wkt() << "\n";
            }
            std::cout << "Compare Segments Ground Truth Again Adapted:\n";
            for (const auto &b_segment: b_segments_list) {
                std::cout << b_segment.wkt() << "\n";
            }
        }

        // convert to vector for further processing
        std::vector<rich_segment_type> a_segments{a_segments_list.cbegin(), a_segments_list.cend()};
        std::vector<rich_segment_type> b_segments{b_segments_list.cbegin(), b_segments_list.cend()};

        // temporary collections
        std::list<rich_segment_type> reverse_a_segments, reverse_b_segments;

        std::size_t i = 0, j = 0;
        while (i < a_segments.size() and j < b_segments.size()) {
            if (geometry::equals_points(a_segments[i].segment.first, b_segments[j].segment.first) and
                geometry::equals_points(a_segments[i].segment.second, b_segments[j].segment.second)) {
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
                    const auto merge = _find_merge(a_segments, b_segments, i, a_segments.size(), j, b_segments.size());

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
                if (geometry::equals_points((*reverse_a_it).segment.first, (*reverse_b_it).segment.first) and
                    geometry::equals_points((*reverse_a_it).segment.second, (*reverse_b_it).segment.second)) {
                    corrects.emplace_back(*reverse_a_it);
                    reverse_a_segments.erase(reverse_a_it);
                    reverse_b_segments.erase(reverse_b_it);
                    reverse_b_it = reverse_b_segments.begin();
                    restart = true;
                    break;
                }
                reverse_a_it++;
            }
            if (not restart) {
                reverse_b_it++;
            }
        }

        // supposed added parts that were part of the ground truth reverse line were in fact correct
        auto error_adds_it = error_adds.begin();
        while (error_adds_it != error_adds.end()) {
            restart = false;
            auto reverse_a_it = reverse_a_segments.begin();
            while (reverse_a_it != reverse_a_segments.end()) {
                if (geometry::equals_points((*reverse_a_it).segment.first, (*error_adds_it).segment.first) and
                    geometry::equals_points((*reverse_a_it).segment.second, (*error_adds_it).segment.second)) {
                    corrects.emplace_back(*reverse_a_it);
                    reverse_a_segments.erase(reverse_a_it);
                    error_adds.erase(error_adds_it);
                    error_adds_it = error_adds.begin();
                    restart = true;
                    break;
                }
                reverse_a_it++;
            }
            if (not restart) {
                error_adds_it++;
            }
        }

        // supposed missed parts that were part of the compare reverse line were in fact correct
        auto error_misses_it = error_misses.begin();
        while (error_misses_it != error_misses.end()) {
            restart = false;
            reverse_b_it = reverse_b_segments.begin();
            while (reverse_b_it != reverse_b_segments.end()) {
                if (geometry::equals_points((*reverse_b_it).segment.first, (*error_misses_it).segment.first) and
                    geometry::equals_points((*reverse_b_it).segment.second, (*error_misses_it).segment.second)) {
                    corrects.emplace_back(*reverse_b_it);
                    reverse_b_segments.erase(reverse_b_it);
                    error_misses.erase(error_misses_it);
                    error_misses_it = error_misses.begin();
                    restart = true;
                    break;
                }
                reverse_b_it++;
            }
            if (not restart) {
                error_misses_it++;
            }
        }

        // remaining duplicate lines are missed lines because already matched duplicate lines were removed above
        for (auto reverse_a_it = reverse_a_segments.begin();
             reverse_a_it != reverse_a_segments.end(); reverse_a_it++) {
            if (std::find_if(error_misses.begin(), error_misses.end(), [&](const auto &error_miss) {
                return geometry::equals_points((*reverse_a_it).segment.first, error_miss.segment.first) and
                       geometry::equals_points((*reverse_a_it).segment.second, error_miss.segment.second);
            }) == error_misses.end()) {
                error_misses.emplace_back(*reverse_a_it);
            }
        }

        // remaining reverse lines were added erroneously
        for (reverse_b_it = reverse_b_segments.begin();
             reverse_b_it != reverse_b_segments.end(); reverse_b_it++) {
            if (std::find_if(error_adds.begin(), error_adds.end(), [&](const auto &error_add) {
                return geometry::equals_points((*reverse_b_it).segment.first, error_add.segment.first) and
                       geometry::equals_points((*reverse_b_it).segment.second, error_add.segment.second);
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
                if (geometry::equals_points((*error_adds_it).segment.first, (*error_misses_it).segment.first) and
                    geometry::equals_points((*error_adds_it).segment.second, (*error_misses_it).segment.second)) {
                    corrects.emplace_back(*error_adds_it);
                    error_adds.erase(error_adds_it);
                    error_misses.erase(error_misses_it);
                    error_misses_it = error_misses.begin();
                    restart = true;
                    break;
                }
                error_adds_it++;
            }
            if (not restart) {
                error_misses_it++;
            }
        }

        // sum lengths
        length_type correct = default_float_type<length_type>::v0;
        auto corrects_it = corrects.begin();
        while (corrects_it != corrects.end()) {
            const auto &correct_segment = *corrects_it;
            length_type length = geometry::default_float_type<length_type>::v0;
            if (correct_segment.has_length) {
                length = correct_segment.length;
            }
            corrects_it++;
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
            if (error_add_segment.has_length) {
                length = error_add_segment.length;
            }
            error_adds_it++;
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
            if (error_miss_segment.has_length) {
                length = error_miss_segment.length;
            }
            error_misses_it++;
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
            error_fraction = ground_truth.has_length and ground_truth.length > default_float_type<length_type>::v0
                             ? ((error_added + error_missed) / ground_truth.length)
                             : (compare.has_length and compare.length > default_float_type<length_type>::v0
                                ? std::numeric_limits<length_type>::infinity()
                                : (error_added + error_missed));
        }

        length_type correct_fraction = default_float_type<length_type>::v0;
        if (error_fraction == default_float_type<length_type>::v0) {
            // when no error has happened, correct fraction is per definition 1.0
            correct_fraction = default_float_type<length_type>::v1;
        } else {
            // when errors have happened, compute correct fraction
            correct_fraction =
                    ground_truth.has_length and ground_truth.length > default_float_type<length_type>::v0
                    ? (compare.has_length and compare.length > default_float_type<length_type>::v0
                       ? (2.0 * correct) / (ground_truth.length + compare.length)
                       : correct / ground_truth.length)
                    : (compare.has_length and compare.length > default_float_type<length_type>::v0
                       ? correct / compare.length
                       : default_float_type<length_type>::v0);
            // in case of rounding errors around 1.0 or 0.0 cut to valid range
            correct_fraction = std::min(default_float_type<length_type>::v1,
                                        std::max(default_float_type<length_type>::v0, correct_fraction));
        }

        return comparison<RichLine>{correct_fraction, error_fraction, correct, error_added, error_missed,
                                    std::move(corrects), std::move(error_adds), std::move(error_misses)};
    }

    template<typename RichLine>
    comparison<RichLine>
    line_comparator<RichLine>::compare(const RichLine &ground_truth, const RichLine &compare) const {
        return this->compare(multi_rich_line_type{ground_truth}, multi_rich_line_type{compare});
    }

    template<typename RichLine>
    typename line_comparator<RichLine>::points_set_type
    line_comparator<RichLine>::_create_points_set(const std::list<rich_segment_type> &segments) const {
        points_set_type points_set{&geometry::point_set_comparator};
        for (const auto &segment: segments) {
            points_set.emplace(segment.segment.first);
            points_set.emplace(segment.segment.second);
        }
        return points_set;
    }

    template<typename RichLine>
    typename line_comparator<RichLine>::rtree_points_type
    line_comparator<RichLine>::_create_points_rtree(const std::list<rich_segment_type> &segments) const {
        points_set_type points_set = _create_points_set(segments);
        rtree_points_type points_index{points_set};
        return points_index;
    }

    template<typename RichLine>
    typename line_comparator<RichLine>::rtree_segments_type
    line_comparator<RichLine>::_create_segments_rtree(std::list<rich_segment_type> &segments) const {
        std::vector<rtree_segments_key_type> segments_list;
        segments_list.reserve(segments.size());
        for (auto it = segments.begin(); it != segments.end(); it++) {
            segments_list.emplace_back(rtree_segments_key_type{(*it).segment, it});
        }
        rtree_segments_type segments_index{segments_list};
        return segments_index;
    }

    template<typename RichLine>
    void line_comparator<RichLine>::_adapt_points(std::list<rich_segment_type> &segments,
                                                  const std::list<rich_segment_type> &adopt_segments,
                                                  const double distance_tolerance) const {
        rtree_points_type points_index = _create_points_rtree(adopt_segments);

        const auto buffer_radius = std::max(1e-3, 2 * distance_tolerance);

        auto adapt_point = [&](auto &segment_it, point_type &point) {
            const auto buffer = geometry::buffer(point, 8, buffer_radius);
            std::vector<point_type> result_points;
            points_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(result_points));

            distance_type nearest_distance = std::numeric_limits<distance_type>::infinity();
            const point_type *nearest_point = nullptr;

            for (const auto &result_point: result_points) {
                if (not geometry::equals_points(point, result_point)) {
                    const auto distance = geometry::distance(point, result_point);
                    if (distance < distance_tolerance and distance < nearest_distance) {
                        nearest_distance = distance;
                        nearest_point = &result_point;
                    }
                }
            }

            if (nearest_point != nullptr) {
                point = *nearest_point;
                (*segment_it)._complete();
                if (not(segment_it == segments.begin() and
                        geometry::equals_points(point, (*segment_it).segment.first)) and
                    std::next(segment_it) != segments.end()) {
                    auto &next_segment = *std::next(segment_it);
                    if (not geometry::equals_points(next_segment.segment.first, point)) {
                        next_segment.segment.first = point;
                        next_segment._complete();
                    }
                }
            }
        };

        for (auto segment_it = segments.begin(); segment_it != segments.end(); segment_it++) {
            auto &segment = *segment_it;

            if (segment_it == segments.begin()) {
                adapt_point(segment_it, segment.segment.first);
            }

            adapt_point(segment_it, segment.segment.second);
        }

        // erase empty strings after adoption
        auto segment_it = segments.begin();
        while (segment_it != segments.end()) {
            auto &segment = *segment_it;
            segment_it++;
            if (not segment.has_length or segment.length == geometry::default_float_type<length_type>::v0) {
                segments.erase(std::prev(segment_it));
            }
        }
    }

    template<typename RichLine>
    void line_comparator<RichLine>::_split_segments(std::list<rich_segment_type> &segments,
                                                    const std::list<rich_segment_type> &compare_segments,
                                                    const double distance_tolerance,
                                                    const double direction_tolerance) const {
        points_set_type points_set = _create_points_set(compare_segments);
        rtree_segments_type segments_index = _create_segments_rtree(segments);

        const auto buffer_radius = std::max(1e-3, 2 * distance_tolerance);

        for (const auto &point: points_set) {
            const auto buffer = geometry::buffer(point, 8, buffer_radius);
            std::vector<rtree_segments_key_type> result_segments;
            segments_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(result_segments));

            auto result_it = result_segments.begin();
            while (result_it != result_segments.end()) {
                result_it++;
                auto &result = *std::prev(result_it);

                const segment_type &segment = result.first;

                if (not geometry::equals_points(segment.first, point) and
                    not geometry::equals_points(segment.second, point)) {
                    const auto distance = geometry::distance(segment, point);

                    if (distance < distance_tolerance) {
                        rich_segment_type from{segment.first, point};
                        rich_segment_type to{point, segment.second};

                        if (from.has_length and to.has_length and from.has_azimuth and to.has_azimuth) {
                            const auto direction = std::fabs(geometry::direction_deg(from.azimuth, to.azimuth));
                            if (direction < direction_tolerance) {
                                auto segment_it = result.second;
                                auto new_segment_it = segments.insert(std::next(segment_it), std::move(to));
                                *segment_it = std::move(from);

                                segments_index.insert(rtree_segments_key_type{(*segment_it).segment, segment_it});
                                segments_index.remove(result);
                                segments_index.insert(
                                        rtree_segments_key_type{(*new_segment_it).segment, new_segment_it});
                            }
                        }
                    }
                }
            }
        }
    }

    template<typename RichLine>
    std::tuple<bool, std::size_t, std::size_t>
    line_comparator<RichLine>::_detect_reverse(std::vector<rich_segment_type> segments,
                                               std::size_t start, std::size_t end) const {
        bool start_found = false, end_found = false;

        if (start - 1 >= 0 and
            geometry::equals_points(segments[start - 1].segment.second, segments[start].segment.first) and
            geometry::equals_points(segments[start - 1].segment.first, segments[start].segment.second)) {
            // segments are equal in reverse, reverse detected

            const auto &start_point = segments[start].segment.first;
            start_found = true;

            for (std::size_t i = start + 1; i < end; ++i) {
                if (geometry::equals_points(segments[i].segment.first, start_point)) {
                    // end point detected
                    end = i;
                    end_found = true;
                }
            }
        }

        return std::tuple{start_found and end_found, start, end};
    }

    template<typename RichLine>
    std::tuple<std::size_t, std::size_t>
    line_comparator<RichLine>::_find_merge(std::vector<rich_segment_type> a_segments,
                                           std::vector<rich_segment_type> b_segments,
                                           std::size_t a_start, std::size_t a_end, std::size_t b_start,
                                           std::size_t b_end) const {
        if (geometry::equals_points(a_segments[a_start].segment.first, b_segments[b_start].segment.first) and
            not geometry::equals_points(a_segments[a_start].segment.second, b_segments[b_start].segment.second)) {
            b_start++;
        }

        for (std::size_t j = b_start; j < b_end; ++j) {
            const auto &b_point = b_segments[j].segment.first;
            for (std::size_t i = a_start; i < a_end; ++i) {
                const auto &a_point = a_segments[i].segment.first;
                if (geometry::equals_points(a_point, b_point)) {
                    return std::tuple{i, j};
                }
            }
        }

        return std::tuple{a_segments.size(), b_segments.size()};
    }

    template
    class line_comparator<rich_line<geometry::types_geographic::line_type>>;

    template
    class line_comparator<rich_line<geometry::types_cartesian::line_type>>;

}
