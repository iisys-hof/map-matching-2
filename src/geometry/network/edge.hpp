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

#ifndef MAP_MATCHING_2_EDGE_HPP
#define MAP_MATCHING_2_EDGE_HPP

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <osmium/osm/types.hpp>

#include "../rich_line.hpp"
#include "tag_helper.hpp"

namespace map_matching_2::geometry::network {

    template<typename Line>
    struct edge : rich_line<Line> {

        using line_type = Line;
        using rich_line_type = rich_line<Line>;
        using point_type = typename boost::geometry::point_type<line_type>::type;
        using segment_type = boost::geometry::model::segment<point_type>;
        using length_type = typename boost::geometry::default_length_result<line_type>::type;
        using angle_type = typename boost::geometry::coordinate_type<line_type>::type;

        std::size_t index;
        osmium::object_id_type id;
        std::vector<osmium::object_id_type> nodes;
        std::vector<std::uint64_t> tags;

        edge();

        edge(osmium::object_id_type id, std::vector<osmium::object_id_type> nodes, Line line,
             std::vector<std::uint64_t> tags = {});

        edge(osmium::object_id_type id, std::vector<osmium::object_id_type> nodes, rich_line_type rich_line,
             std::vector<std::uint64_t> tags = {});

        ~edge() = default;

        edge(const edge &other) = default;

        edge(edge &&other) noexcept = default;

        edge &operator=(const edge &other) = default;

        edge &operator=(edge &&other) noexcept = default;

        [[nodiscard]] static edge merge(edge &a, edge &b);

        [[nodiscard]] std::vector<std::string> header() const;

        [[nodiscard]] std::vector<std::string> row(const tag_helper &tag_helper) const;

        [[nodiscard]] std::string str(const tag_helper &tag_helper) const;

        template<typename LineT>
        friend std::ostream &operator<<(std::ostream &out, const edge<LineT> &edge);

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_line_type>(*this);
            ar & index;
            ar & id;
            ar & nodes;
            ar & tags;
        }

    };

}

#endif //MAP_MATCHING_2_EDGE_HPP
