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

    template<typename RichLine>
    struct edge : public RichLine {

        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        osmium::object_id_type id;
        std::vector<std::uint64_t> tags;

        template<typename RichLineT = RichLine,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        edge()
                :  rich_line_type{}, id{}, tags{} {}

        edge(osmium::object_id_type id, const line_type &line, std::vector<std::uint64_t> tags = {})
                : rich_line_type{line}, id{id}, tags{std::move(tags)} {}

        edge(osmium::object_id_type id, rich_line_type rich_line, std::vector<std::uint64_t> tags = {})
                : rich_line_type{std::move(rich_line)}, id{id}, tags{std::move(tags)} {}

        ~edge() = default;

        edge(const edge &other)
                : rich_line_type{static_cast<const rich_line_type &>(other)}, id{other.id}, tags{other.tags} {}

        template<typename RichLineT>
        edge(const edge<RichLineT> &other)
                : rich_line_type{static_cast<const RichLineT &>(other)}, id{other.id}, tags{other.tags} {}

        edge(edge &&other) noexcept
                : rich_line_type{std::move(static_cast<rich_line_type &&>(other))}, id{other.id},
                  tags{std::move(other.tags)} {}

        template<typename RichLineT>
        edge(edge<RichLineT> &&other) noexcept
                : rich_line_type{std::move(static_cast<RichLineT &&>(other))}, id{other.id},
                  tags{std::move(other.tags)} {}

        edge &operator=(const edge &other) {
            rich_line_type::operator=(static_cast<const rich_line_type &>(other));
            id = other.id;
            tags = other.tags;
            return *this;
        }

        template<typename RichLineT>
        edge &operator=(const edge <RichLineT> &other) {
            rich_line_type::operator=(static_cast<const RichLineT &>(other));
            id = other.id;
            tags = other.tags;
            return *this;
        }

        edge &operator=(edge &&other) noexcept {
            rich_line_type::operator=(std::move(static_cast<rich_line_type &&>(other)));
            id = other.id;
            tags = std::move(other.tags);
            return *this;
        }

        template<typename RichLineT>
        edge &operator=(edge <RichLineT> &&other) noexcept {
            rich_line_type::operator=(std::move(static_cast<RichLineT &&>(other)));
            id = other.id;
            tags = std::move(other.tags);
            return *this;
        }

        template<typename Edge>
        [[nodiscard]] static Edge merge(Edge &a, Edge &b) {
            if (b.rich_segments().empty()) {
                return Edge{a};
            } else if (not b.rich_segments().empty() and a.rich_segments().empty()) {
                return Edge{b};
            } else {
                assert(geometry::equals_points(a.rich_segments().back().second(), b.rich_segments().front().first()));

                using rich_line_type = typename Edge::rich_line_type;
                return Edge{a.id, rich_line_type::template merge<rich_line_type>({a, b}), a.tags};
            }
        }

        [[nodiscard]] std::vector<std::string> header() const {
            return {"id", "nodes", "geometry", "length", "tags"};
        }

        [[nodiscard]] std::vector<std::string> row(const tag_helper &tag_helper) const {
            std::vector<std::string> row;
            row.reserve(4);
            row.emplace_back(std::to_string(id));
            row.emplace_back(this->wkt());
            if constexpr (geometry::is_rich_line_v<RichLine>) {
                if (this->has_length()) {
                    row.emplace_back(std::to_string(this->length()));
                } else {
                    row.emplace_back("NaN");
                }
            } else {
                row.emplace_back("NaN");
            }
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

        friend bool operator==(const edge &left, const edge &right) {
            return left.id == right.id and static_cast<rich_line_type>(left) == static_cast<rich_line_type>(right);
        }

        friend bool operator!=(const edge &left, const edge &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const edge &edge) {
            return out << edge.str(tag_helper{});
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_line_type>(*this);
            ar & id;
            ar & tags;
        }

    };

    template<typename Point>
    using edge_type = edge<rich_line_type < Point>>;

    template<typename Point>
    using eager_edge_type = edge<eager_rich_line_type < Point>>;

    template<typename Point>
    using lazy_edge_type = edge<lazy_rich_line_type < Point>>;

    template<typename RichLine>
    struct internal_edge : public edge<RichLine> {

        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        std::size_t index;

        template<typename RichLineT = RichLine,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        internal_edge()
                :  edge<RichLine>{}, index{std::numeric_limits<std::size_t>::max()} {}

        internal_edge(osmium::object_id_type id, const line_type &line, std::vector<std::uint64_t> tags = {})
                : edge<RichLine>{id, line, std::move(tags)}, index{std::numeric_limits<std::size_t>::max()} {}

        internal_edge(osmium::object_id_type id, rich_line_type rich_line, std::vector<std::uint64_t> tags = {})
                : edge<RichLine>{id, std::move(rich_line), std::move(tags)},
                  index{std::numeric_limits<std::size_t>::max()} {}

        ~internal_edge() = default;

        internal_edge(const internal_edge &other)
                : edge<rich_line_type>{other}, index{other.index} {}

        template<typename RichLineT>
        internal_edge(const internal_edge<RichLineT> &other)
                : edge<rich_line_type>{other}, index{other.index} {}

        internal_edge(internal_edge &&other) noexcept
                : edge<rich_line_type>{std::move(other)}, index{other.index} {}

        template<typename RichLineT>
        internal_edge(internal_edge<RichLineT> &&other) noexcept
                : edge<rich_line_type>{std::move(other)}, index{other.index} {}

        internal_edge &operator=(const internal_edge &other) {
            edge<rich_line_type>::operator=(other);
            index = other.index;
            return *this;
        }

        template<typename RichLineT>
        internal_edge &operator=(const internal_edge<RichLineT> &other) {
            edge<rich_line_type>::operator=(other);
            index = other.index;
            return *this;
        }

        internal_edge &operator=(internal_edge &&other) noexcept {
            edge<rich_line_type>::operator=(std::move(other));
            index = other.index;
            return *this;
        }

        template<typename RichLineT>
        internal_edge &operator=(internal_edge<RichLineT> &&other) noexcept {
            edge<rich_line_type>::operator=(std::move(other));
            index = other.index;
            return *this;
        }

        [[nodiscard]] static internal_edge merge(internal_edge &a, internal_edge &b) {
            return edge<rich_line_type>::merge(a, b);
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<edge<RichLine>>(*this);
            ar & index;
        }

    };

    template<typename Point>
    using internal_edge_type = internal_edge<rich_line_type < Point>>;

    template<typename Point>
    using eager_internal_edge_type = internal_edge<eager_rich_line_type < Point>>;

    template<typename Point>
    using lazy_internal_edge_type = internal_edge<lazy_rich_line_type < Point>>;

}

#endif //MAP_MATCHING_2_EDGE_HPP
