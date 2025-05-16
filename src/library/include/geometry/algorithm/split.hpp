// Copyright (C) 2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_SPLIT_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_SPLIT_HPP

#include <cstddef>

#include "geometry/rich_type/traits/rich_line.hpp"
#include "geometry/rich_type/traits/multi_rich_line.hpp"

#include "types/geometry/point.hpp"

namespace map_matching_2::geometry {

    template<is_multi_rich_line MultiRichLine, typename Callback>
    void multi_rich_line_time_split(const MultiRichLine &multi_rich_line, const double split_time,
            const Callback &callback) requires
        (is_time_point<typename geometry::multi_rich_line_traits<MultiRichLine>::point_type>) {
        using time_point_type = typename geometry::multi_rich_line_traits<MultiRichLine>::point_type;

        bool split = false;
        std::size_t start_outer = 0;
        std::size_t start_inner = 0;
        const time_point_type *curr = nullptr;
        const time_point_type *prev = nullptr;
        for (std::size_t i = 0; i < multi_rich_line.size(); ++i) {
            const auto &rich_line = multi_rich_line.at(i);
            for (std::size_t j = 0; j < rich_line.size(); ++j) {
                curr = &rich_line.at(j);
                if (curr and prev) {
                    double time_difference = curr->timestamp() - prev->timestamp();
                    if (time_difference >= split_time) {
                        callback(start_outer, start_inner, j, i + 1);
                        start_outer = i;
                        start_inner = j;
                        split = true;
                    }
                }
                prev = curr;
            }
        }

        if (split) {
            callback(start_outer, start_inner, multi_rich_line.back().size(), multi_rich_line.size());
        }
    }

    template<is_multi_rich_line MultiRichLine>
    MultiRichLine multi_rich_line_time_split(const MultiRichLine &multi_rich_line, const double split_time) requires
        (is_time_point<typename geometry::multi_rich_line_traits<MultiRichLine>::point_type>) {
        using multi_rich_line_type = MultiRichLine;
        using rich_lines_container_type = typename geometry::multi_rich_line_traits<
            multi_rich_line_type>::rich_lines_container_type;

        rich_lines_container_type container;
        multi_rich_line_time_split(multi_rich_line, split_time, [&](
                const std::size_t from_outer, const std::size_t from_inner,
                const std::size_t to_inner, const std::size_t to_outer) {
                    auto _multi_rich_line = multi_rich_line.template extract<multi_rich_line_type>(
                            from_outer, from_inner, to_inner, to_outer);
                    std::move(std::begin(_multi_rich_line.rich_lines()), std::end(_multi_rich_line.rich_lines()),
                            std::back_inserter(container));
                });

        if (container.empty()) {
            // no split
            return multi_rich_line;
        } else {
            return multi_rich_line_type{std::move(container)};
        }
    }

    template<is_rich_line RichLine, typename Callback>
    void rich_line_time_split(const RichLine &rich_line, const double split_time,
            const Callback &callback) requires
        (is_time_point<typename geometry::rich_line_traits<RichLine>::point_type>) {
        using time_point_type = typename geometry::rich_line_traits<RichLine>::point_type;

        bool split = false;
        std::size_t start = 0;
        const time_point_type *curr = nullptr;
        const time_point_type *prev = nullptr;

        for (std::size_t i = 0; i < rich_line.size(); ++i) {
            curr = &rich_line.at(i);
            if (curr and prev) {
                double time_difference = curr->timestamp() - prev->timestamp();
                if (time_difference >= split_time) {
                    callback(start, i);
                    start = i;
                    split = true;
                }
            }
            prev = curr;
        }

        if (split) {
            callback(start, rich_line.size());
        } else {
            callback(0, rich_line.size());
        }
    }

    template<is_multi_rich_line MultiRichLine>
    MultiRichLine rich_line_time_split(
            const typename geometry::multi_rich_line_traits<MultiRichLine>::rich_line_type &rich_line,
            const double split_time) requires
        (is_time_point<typename geometry::multi_rich_line_traits<MultiRichLine>::point_type>) {
        using multi_rich_line_type = MultiRichLine;
        using rich_lines_container_type = typename geometry::multi_rich_line_traits<
            multi_rich_line_type>::rich_lines_container_type;

        rich_lines_container_type container;
        rich_line_time_split(rich_line, split_time, [&](const std::size_t from, const std::size_t to) {
            container.emplace_back(rich_line.sub_rich_line(from, to));
        });

        if (container.empty()) {
            // no split
            return multi_rich_line_type{rich_line};
        } else {
            return multi_rich_line_type{std::move(container)};
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_SPLIT_HPP
