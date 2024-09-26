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

#ifndef MAP_MATCHING_2_GEOMETRY_ALGORITHM_MEDIAN_HPP
#define MAP_MATCHING_2_GEOMETRY_ALGORITHM_MEDIAN_HPP

#include <cstddef>
#include <algorithm>
#include <vector>

#include "geometry/traits.hpp"

namespace map_matching_2::geometry {

    template<template<typename, typename> typename Container,
        typename Element,
        template<typename> typename Allocator>
    [[nodiscard]] Element median(Container<Element, Allocator<Element>> &container) {
        assert(not container.empty());

        std::size_t middle = container.size() / 2;
        std::nth_element(container.begin(), container.begin() + middle, container.end());
        if (container.size() % 2 == 1) {
            return container[middle];
        } else {
            std::nth_element(container.begin(), container.begin() + middle - 1, container.end());
            return (container[middle - 1] + container[middle]) / static_cast<Element>(2);
        }
    }

    template<template<typename, typename> typename Container,
        typename Element,
        template<typename> typename Allocator>
    [[nodiscard]] Element median_absolute_deviation(Element center, Container<Element, Allocator<Element>> &container) {
        assert(not container.empty());

        for (Element &element : container) {
            element = std::abs(element - center);
        }
        return median(container);
    }

    template<template<typename, typename> typename Container,
        typename Element,
        template<typename> typename Allocator>
    [[nodiscard]] Element median_absolute_deviation(Container<Element, Allocator<Element>> &container) {
        return median_absolute_deviation(median(container), container);
    }

    template<template<typename, typename> typename Container,
        typename Point,
        template<typename> typename Allocator>
    [[nodiscard]] Point median_point(const Container<Point, Allocator<Point>> &points) {
        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));
        assert(not points.empty());
        assert(boost::geometry::dimension<Point>::value >= 1 and boost::geometry::dimension<Point>::value <= 3);

        using coordinate_type = typename data<Point>::coordinate_type;

        std::vector<coordinate_type> x, y, z;
        if constexpr (boost::geometry::dimension<Point>::value >= 1) {
            x.reserve(points.size());
        }
        if constexpr (boost::geometry::dimension<Point>::value >= 2) {
            y.reserve(points.size());
        }
        if constexpr (boost::geometry::dimension<Point>::value == 3) {
            z.reserve(points.size());
        }

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            for (const auto &point : points) {
                x.emplace_back(boost::geometry::get<0>(point));
            }
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            for (const auto &point : points) {
                x.emplace_back(boost::geometry::get<0>(point));
                y.emplace_back(boost::geometry::get<1>(point));
            }
        } else {
            for (const auto &point : points) {
                x.emplace_back(boost::geometry::get<0>(point));
                y.emplace_back(boost::geometry::get<1>(point));
                z.emplace_back(boost::geometry::get<2>(point));
            }
        }

        coordinate_type median_x, median_y, median_z;
        if constexpr (boost::geometry::dimension<Point>::value >= 1) {
            median_x = median(x);
        }
        if constexpr (boost::geometry::dimension<Point>::value >= 2) {
            median_y = median(y);
        }
        if constexpr (boost::geometry::dimension<Point>::value == 3) {
            median_z = median(z);
        }

        if constexpr (boost::geometry::dimension<Point>::value == 1) {
            return Point{median_x};
        } else if constexpr (boost::geometry::dimension<Point>::value == 2) {
            return Point{median_x, median_y};
        } else {
            return Point{median_x, median_y, median_z};
        }
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_ALGORITHM_MEDIAN_HPP
