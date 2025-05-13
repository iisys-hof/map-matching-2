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

#ifndef MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP
#define MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP

#include <vector>

#include <boost/unordered/unordered_flat_set.hpp>

#include "geometry/rich_type/concepts.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/azimuth.hpp"

#include "matching/defect.hpp"

namespace map_matching_2::matching {

    template<geometry::is_rich_line RichLine>
    class detector {

    public:
        using rich_line_type = RichLine;
        using set_type = boost::unordered_flat_set<defect>;

        [[nodiscard]] static std::vector<set_type> detect(
                const rich_line_type &rich_line, const bool detect_duplicates = true,
                const bool detect_forward_backward = false) {
            std::vector<set_type> defects(rich_line.size(), set_type{});

            if (detect_duplicates) {
                for (std::size_t from = 0, to = 1; to < rich_line.size(); ++from, ++to) {
                    if (_equal(rich_line, from, to)) {
                        defects[to].emplace(defect::equal);
                    }
                }
            }

            if (detect_forward_backward) {
                for (std::int64_t from = 0, to = 1; to < rich_line.size(); ++from, ++to) {
                    while (to < rich_line.size() and
                        (defects[to].contains(defect::forward_backward) or
                            defects[to].contains(defect::equal))) {
                        ++to;
                    }
                    if (_forward_backward(rich_line, from, to)) {
                        defects[to].emplace(defect::forward_backward);
                        --from;
                        --to;
                    } else if (from + 1 != to) {
                        from = to - 1;
                    }
                }
            }

            for (auto &defect_set : defects) {
                if (defect_set.empty()) {
                    defect_set.emplace(defect::none);
                }
            }

            // debug output
            // for (std::size_t i = 0; i < defects.size(); ++i) {
            //     const auto &defect_set = defects.at(i);
            //     if (not defect_set.contains(defect::none)) {
            //         std::cout << "Defect at " << i << ": " << defect_set << std::endl;
            //     }
            // }

            return defects;
        }

        // [[nodiscard]] static set_type detect(
        //         const rich_line_type &rich_line, const std::size_t from, const std::size_t to,
        //         const bool detect_forward_backward = false) {
        //     set_type defect_set;
        //
        //     if (detect_forward_backward and _forward_backward(rich_line, from, to)) {
        //         defect_set.emplace(defect::forward_backward);
        //     }
        //
        //     if (defect_set.empty()) {
        //         defect_set.emplace(defect::none);
        //     }
        //
        //     return defect_set;
        // }

        static void remove_defects(rich_line_type &rich_line, const std::vector<set_type> &defects) {
            std::vector<std::size_t> remove;
            for (std::size_t i = 0; i < defects.size(); ++i) {
                const auto &defect_set = defects[i];
                if (not defect_set.contains(defect::none)) {
                    remove.emplace_back(i);
                }
            }

            rich_line.erase(remove);
        }

    private:
        bool static _equal(const rich_line_type &rich_line, const std::size_t from, const std::size_t to) {
            if (from >= rich_line.size() or to >= rich_line.size()) {
                return false;
            }

            return geometry::equals_points(rich_line.at(from), rich_line.at(to));
        }

        bool static _forward_backward(const rich_line_type &rich_line, const std::size_t from, const std::size_t to,
                const double relative_difference = 0.8) {
            if (from >= rich_line.size() or to >= rich_line.size()) {
                return false;
            }

            const std::int64_t prev = static_cast<std::int64_t>(from) - 1;
            if (prev < 0) {
                return false;
            }

            bool adjacent_from = prev + 1 == from;
            const auto current_azimuth =
                    adjacent_from
                    ? rich_line.azimuth(prev)
                    : geometry::azimuth_deg(rich_line.at(prev), rich_line.at(from));
            bool adjacent_to = from + 1 == to;
            const auto next_azimuth =
                    adjacent_to
                    ? rich_line.azimuth(from)
                    : geometry::azimuth_deg(rich_line.at(from), rich_line.at(to));
            const auto diff_azimuth = geometry::angle_diff(current_azimuth, next_azimuth);

            const auto relative_probability = std::fabs(diff_azimuth) / 180.0;
            assert(relative_probability >= 0.0 and relative_probability <= 1.0);

            return relative_probability >= relative_difference;
        }

    };

}

#endif //MAP_MATCHING_2_MATCHING_ALGORITHM_DETECT_HPP
