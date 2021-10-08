// Copyright (C) 2020-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_NODE_HPP
#define MAP_MATCHING_2_NODE_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <osmium/osm/types.hpp>

#include "tag_helper.hpp"

namespace map_matching_2::geometry::network {

    template<typename Point>
    struct node {

        using point_type = Point;

        std::size_t index;
        osmium::object_id_type id;
        Point point;
        std::vector<std::uint64_t> tags;

        node();

        node(osmium::object_id_type id, Point point, std::vector<std::uint64_t> tags = {});

        ~node() = default;

        node(const node &other) = default;

        node(node &&other) noexcept = default;

        node &operator=(const node &other) = default;

        node &operator=(node &&other) noexcept = default;

        [[nodiscard]] std::string wkt() const;

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row(const tag_helper &tag_helper) const;

        [[nodiscard]] std::string str(const tag_helper &tag_helper) const;

        template<typename PointT>
        friend std::ostream &operator<<(std::ostream &out, const node<PointT> &node);

    };

}

#endif //MAP_MATCHING_2_NODE_HPP
