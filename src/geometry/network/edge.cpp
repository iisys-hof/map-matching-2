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

#include "edge.hpp"

#include <geometry/util.hpp>

#include "../types.hpp"

namespace map_matching_2::geometry::network {

    template<typename Line>
    edge<Line>::edge() : edge({}, {}, Line{}) {}

    template<typename Line>
    edge<Line>::edge(osmium::object_id_type id, std::vector<osmium::object_id_type> nodes, Line line,
                     std::vector<std::uint64_t> tags)
            : rich_line_type{std::move(line)}, index{std::numeric_limits<std::size_t>::max()}, id{id},
              nodes{std::move(nodes)}, tags{std::move(tags)} {}

    template<typename Line>
    edge<Line>::edge(osmium::object_id_type id, std::vector<osmium::object_id_type> nodes, rich_line_type rich_line,
                     std::vector<std::uint64_t> tags)
            : rich_line_type{std::move(rich_line)}, index{std::numeric_limits<std::size_t>::max()}, id{id},
              nodes{std::move(nodes)}, tags{std::move(tags)} {}

    template<typename Line>
    edge<Line> edge<Line>::merge(edge &a, edge &b) {
        if (b.line().empty()) {
            return edge{a};
        } else if (not b.line().empty() and a.line().empty()) {
            return edge{b};
        } else {
            assert(geometry::equals_points(a.line().back(), b.line().front()));

            std::vector<osmium::object_id_type> nodes;
            nodes.reserve(a.nodes.size() + b.nodes.size() - 1);
            std::move(a.nodes.begin(), a.nodes.end(), std::back_inserter(nodes));
            std::move(std::next(b.nodes.begin()), b.nodes.end(), std::back_inserter(nodes));

            rich_line_type rich_line = rich_line_type::merge({a, b});

            return edge{a.id, std::move(nodes), std::move(rich_line), a.tags};
        }
    }

    template<typename Line>
    std::vector<std::string> edge<Line>::header() const {
        return {"id", "nodes", "geometry", "length", "tags"};
    }

    template<typename Line>
    std::vector<std::string> edge<Line>::row(const tag_helper &tag_helper) const {
        std::vector<std::string> row;
        row.reserve(5);
        row.emplace_back(std::to_string(id));
        std::string nodes_str;
        nodes_str.append("[");
        bool first = true;
        for (const auto &node_id: nodes) {
            if (!first) {
                nodes_str.append(",");
            }
            first = false;
            nodes_str.append(std::to_string(node_id));
        }
        nodes_str.append("]");
        row.emplace_back(std::move(nodes_str));
        row.emplace_back(this->wkt());
        row.emplace_back(std::to_string(this->length()));
        std::string tags_str;
        if (!tags.empty()) {
            tags_str.append("[");
            first = true;
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

    template<typename Line>
    [[nodiscard]] std::string edge<Line>::str(const tag_helper &tag_helper) const {
        const auto &fields = row(tag_helper);
        std::string str = "Edge(";
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

    template<typename Line>
    std::ostream &operator<<(std::ostream &out, const edge<Line> &edge) {
        out << edge.str(tag_helper{});
        return out;
    }

    template
    class edge<geometry::types_geographic::line_type>;

    template
    class edge<geometry::types_cartesian::line_type>;

    template
    std::ostream &operator<<(std::ostream &out, const edge <geometry::types_geographic::line_type> &edge);

    template
    std::ostream &operator<<(std::ostream &out, const edge <geometry::types_cartesian::line_type> &edge);

}
