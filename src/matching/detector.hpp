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

#ifndef MAP_MATCHING_2_DETECTOR_HPP
#define MAP_MATCHING_2_DETECTOR_HPP

#include <iostream>
#include <vector>

#include <absl/container/btree_set.h>

#include <geometry/util.hpp>

namespace map_matching_2::matching {

    enum defect {
        none,
        equal,
        forward_backward
    };

    std::ostream &operator<<(std::ostream &out, const absl::btree_set<defect> &defect_set) {
        std::string defect_str;
        if (defect_set.contains(defect::none)) {
            defect_str.append("none, ");
        }
        if (defect_set.contains(defect::equal)) {
            defect_str.append("equal, ");
        }
        if (defect_set.contains(defect::forward_backward)) {
            defect_str.append("forward_backward, ");
        }
        if (defect_str.length() > 0 and defect_str.ends_with(", ")) {
            defect_str.erase(defect_str.length() - 2);
        }
        out << defect_str;
        return out;
    }

    template<typename Track>
    class detector {

    public:

        using measurement_type = typename Track::measurement_type;

        [[nodiscard]] std::vector<absl::btree_set<defect>> detect(
                const Track &track, bool detect_duplicates = true, bool detect_forward_backward = true) const {
            std::vector<absl::btree_set<defect>> defects;
            defects.reserve(track.size());
            for (std::size_t i = 0; i < track.size(); ++i) {
                defects.emplace_back(absl::btree_set<defect>{defect::none});
            }

            if (detect_duplicates) {
                for (std::int64_t from = 0, to = 1; to < track.size(); ++from, ++to) {
                    if (_equal(track, from, to)) {
                        defects[to].emplace(defect::equal);
                    }
                }
            }

            if (detect_forward_backward) {
                for (std::int64_t from = 0, to = 1; to < track.size(); ++from, ++to) {
                    while (to < track.size() and
                           (defects[to].contains(defect::forward_backward) or
                            defects[to].contains(defect::equal))) {
                        ++to;
                    }
                    if (_forward_backward(track, from, to)) {
                        defects[to].emplace(defect::forward_backward);
                        --from;
                        --to;
                    } else if (from + 1 != to) {
                        from = to - 1;
                    }
                }
            }

            for (auto &defect_set: defects) {
                if (defect_set.size() > 1) {
                    defect_set.erase(defect::none);
                }
            }

            // debug output
//            for (std::size_t i = 0; i < defects.size(); ++i) {
//                std::set<defect> defect_set = defects.at(i);
//                if (not defect_set.contains(defect::none)) {
//                    std::cout << "Defect at " << i << ": " << defect_set << std::endl;
//                }
//            }

            return defects;
        }

        [[nodiscard]] absl::btree_set<defect> detect(
                const Track &track, std::size_t from, std::size_t to, bool detect_forward_backward = true) const {
            absl::btree_set<defect> defect_set;

            if (detect_forward_backward and _forward_backward(track, from, to)) {
                defect_set.emplace(defect::forward_backward);
            }

            if (defect_set.empty()) {
                defect_set.emplace(defect::none);
            }

            return defect_set;
        }

    private:
        bool _equal(const Track &track, std::size_t from, std::size_t to) const {
            if (from >= track.size() or to >= track.size()) {
                return false;
            }

            return geometry::equals_points(track.at(from), track.at(to));
        }

        bool _forward_backward(const Track &track, std::size_t from, std::size_t to,
                               double relative_difference = 0.8) const {
            if (from >= track.size() or to >= track.size()) {
                return false;
            }

            const std::int64_t prev = ((std::int64_t) from) - 1;
            if (prev < 0) {
                return false;
            }

            bool adjacent_from = prev + 1 == from;
            const auto current_azimuth =
                    adjacent_from ? track.rich_segments()[prev].azimuth()
                                  : geometry::azimuth_deg(track.at(prev), track.at(from));
            bool adjacent_to = from + 1 == to;
            const auto next_azimuth =
                    adjacent_to ? track.rich_segments()[from].azimuth()
                                : geometry::azimuth_deg(track.at(from), track.at(to));
            const auto diff_azimuth = geometry::angle_diff(current_azimuth, next_azimuth);

            const auto relative_probability = std::fabs(diff_azimuth) / 180.0;
            assert(relative_probability >= 0.0 and relative_probability <= 1.0);

            return relative_probability >= relative_difference;
        }

    };

}

#endif //MAP_MATCHING_2_DETECTOR_HPP
