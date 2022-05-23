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

#include "node.hpp"

#include "../util.hpp"
#include "../types.hpp"

namespace map_matching_2::geometry::network {

    template<typename Point>
    node<Point>::node() : node({}, {}) {}

    template<typename Point>
    node<Point>::node(osmium::object_id_type id, Point point, std::vector<std::uint64_t> tags)
            : index{std::numeric_limits<std::size_t>::max()}, id{id},
              point{std::move(point)}, tags{std::move(tags)} {
    }

    template<typename Point>
    std::string node<Point>::wkt() const {
        return geometry::to_wkt(point);
    }

    template<typename Point>
    std::vector<std::string> node<Point>::header() const {
        return {"id", "geometry", "tags"};
    }

    template<typename Point>
    std::vector<std::string> node<Point>::row(const tag_helper &tag_helper) const {
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

    template<typename Point>
    std::string node<Point>::str(const tag_helper &tag_helper) const {
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

    template<typename Point>
    std::ostream &operator<<(std::ostream &out, const node<Point> &node) {
        out << node.str(tag_helper{});
        return out;
    }

    template
    class node<geometry::types_geographic::point_type>;

    template
    class node<geometry::types_cartesian::point_type>;

    template
    std::ostream &operator<<(std::ostream &out, const node <geometry::types_geographic::point_type> &node);

    template
    std::ostream &operator<<(std::ostream &out, const node <geometry::types_cartesian::point_type> &node);

}
