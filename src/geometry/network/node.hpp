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

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <osmium/osm/types.hpp>

#include "../util.hpp"
#include "tag_helper.hpp"

namespace map_matching_2::geometry::network {

    template<typename Point>
    struct node {

        using point_type = Point;

        osmium::object_id_type id;
        Point point;
        std::vector<std::uint64_t> tags;

        node()
                : node({}, {}) {}

        node(osmium::object_id_type id, Point point, std::vector<std::uint64_t> tags = {})
                : id{id}, point{std::move(point)}, tags{std::move(tags)} {}

        ~node() = default;

        node(const node &other) = default;

        node(node &&other) noexcept = default;

        node &operator=(const node &other) = default;

        node &operator=(node &&other) noexcept = default;

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(point);
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"id", "geometry", "tags"};
        }

        [[nodiscard]] std::vector<std::string> row(const tag_helper &tag_helper) const {
            std::vector<std::string> row;
            row.reserve(3);
            row.emplace_back(std::to_string(id));
            row.emplace_back(wkt());
            std::string tags_str;
            if (!tags.empty()) {
                tags_str.append("[");
                bool first = true;
                for (const auto &tag_id: tags) {
                    if (!first) {
                        tags_str.append(",");
                    }
                    first = false;
                    try {
                        const auto &tag = tag_helper.tag(tag_id);
                        tags_str.append("'").append(tag.first).append("':'").append(tag.second).append("'");
                    } catch (std::invalid_argument &) {}
                }
                tags_str.append("]");
            }
            if (tags_str.length() <= 2) {
                tags_str.clear();
            }
            row.emplace_back(std::move(tags_str));
            return row;
        }

        [[nodiscard]] std::string str(const tag_helper &tag_helper) const {
            const auto &fields = row(tag_helper);
            std::string str = "Node(";
            bool first = true;
            for (const auto &field: fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        friend bool operator==(const node &left, const node &right) {
            return left.id == right.id and geometry::equals_points(left.point, right.point);
        }

        friend bool operator!=(const node &left, const node &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const node &node) {
            return out << node.str(tag_helper{});
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & id;
            ar & point;
            ar & tags;
        }

    };

    template<typename Point>
    struct internal_node : public node<Point> {

        using point_type = Point;

        std::size_t index;

        internal_node()
                : node<Point>() {}

        internal_node(osmium::object_id_type id, Point point, std::vector<std::uint64_t> tags = {})
                : node<Point>(id, std::move(point), std::move(tags)),
                  index{std::numeric_limits<std::size_t>::max()} {}

        ~internal_node() = default;

        internal_node(const internal_node &other) = default;

        internal_node(internal_node &&other) noexcept = default;

        internal_node &operator=(const internal_node &other) = default;

        internal_node &operator=(internal_node &&other) noexcept = default;

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<node<Point>>(*this);
            ar & index;
        }

    };

}

#endif //MAP_MATCHING_2_NODE_HPP
