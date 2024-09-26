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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_EDGE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_EDGE_HPP

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <osmium/osm/types.hpp>

#include "io/helper/tag_helper.hpp"

namespace map_matching_2::geometry::network {

    template<typename RichLine,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    struct edge {

        using rich_line_type = RichLine;
        using tag_type = std::uint64_t;

        using allocator_type = Allocator<void>;

        using tags_container_type = Container<tag_type, Allocator<tag_type>>;

        osmium::object_id_type id{};
        rich_line_type rich_line;
        tags_container_type tags;

        constexpr edge() requires
            (std::default_initializable<rich_line_type> and std::default_initializable<tags_container_type> and
                std::default_initializable<allocator_type>)
            : rich_line{}, tags{} {}

        constexpr edge(osmium::object_id_type id, rich_line_type rich_line, tags_container_type tags = {})
            : id{id}, rich_line{std::move(rich_line)}, tags{std::move(tags)} {}

        constexpr edge(const edge &other)
            : id{other.id}, rich_line{other.rich_line}, tags{other.tags} {}

        constexpr edge(const edge &other, const allocator_type &allocator)
            : id{other.id}, rich_line{other.rich_line, allocator}, tags{other.tags, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr edge(const edge<rich_line_type, ContainerT, AllocatorT> &other)
            : id{other.id}, rich_line{other.rich_line}, tags{std::cbegin(other.tags), std::cend(other.tags)} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr edge(const edge<rich_line_type, ContainerT, AllocatorT> &other, const allocator_type &allocator)
            : id{other.id}, rich_line{other.rich_line, allocator},
            tags{std::cbegin(other.tags), std::cend(other.tags), allocator} {}

        constexpr edge(edge &&other) noexcept
            : id{std::move(other.id)}, rich_line{std::move(other.rich_line)}, tags{std::move(other.tags)} {}

        constexpr edge &operator=(const edge &other) {
            if (this != &other) {
                id = other.id;
                rich_line = other.rich_line;
                tags = other.tags;
            }
            return *this;
        }

        constexpr edge &operator=(edge &&other) noexcept {
            if (this != &other) {
                id = std::move(other.id);
                rich_line = std::move(other.rich_line);
                tags = std::move(other.tags);
            }
            return *this;
        }

        constexpr ~edge() = default;

        [[nodiscard]] static edge merge(edge &&a, edge &&b) {
            if (b.rich_line.empty()) {
                return edge{std::move(a)};
            } else if (not b.rich_line.empty() and a.rich_line.empty()) {
                return edge{std::move(b)};
            } else {
                _merge_tags(a, b);
                return edge{a.id, std::move(a.rich_line.merge(std::move(b.rich_line))), std::move(a.tags)};
            }
        }

        static void _merge_tags(edge &a, edge &b) {
            std::vector<tag_type> _missing_tags;
            _missing_tags.reserve(b.tags.size());
            std::remove_copy_if(b.tags.cbegin(), b.tags.cend(), std::back_inserter(_missing_tags),
                    [&a](tag_type tag) {
                        return std::find(a.tags.begin(), a.tags.end(), tag) != a.tags.end();
                    });
            a.tags.reserve(a.tags.size() + _missing_tags.size());
            std::copy(_missing_tags.begin(), _missing_tags.end(), std::back_inserter(a.tags));
        }

        [[nodiscard]] static std::vector<std::string> header() {
            return {"id", "geometry", "length", "tags"};
        }

        template<typename TagHelper>
        [[nodiscard]] std::vector<std::string> row(const TagHelper &tag_helper) const {
            std::vector<std::string> row;
            row.reserve(4);
            row.emplace_back(std::to_string(id));
            row.emplace_back(rich_line.wkt());
            if constexpr (geometry::is_rich_line<rich_line_type>) {
                if (rich_line.has_length()) {
                    row.emplace_back(std::to_string(rich_line.length()));
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
                for (const auto &tag_id : tags) {
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

        template<typename TagHelper>
        [[nodiscard]] std::string str(const TagHelper &tag_helper) const {
            const auto &fields = row(tag_helper);
            std::string str = "Edge(";
            bool first = true;
            for (const auto &field : fields) {
                if (!first) {
                    str.append(",");
                }
                first = false;
                str.append(field);
            }
            str.append(")");
            return str;
        }

        [[nodiscard]] std::string str() const {
            return str(io::helper::tag_helper_dummy{});
        }

        friend constexpr bool operator==(const edge &left, const edge &right) {
            return left.id == right.id and
                    static_cast<rich_line_type>(left.rich_line) == static_cast<rich_line_type>(right.rich_line);
        }

        friend constexpr bool operator!=(const edge &left, const edge &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const edge &edge) {
            return out << edge.str();
        }

        friend std::size_t hash_value(const edge &data) {
            std::size_t seed = 0;
            boost::hash_combine(seed, data.id);
            boost::hash_combine(seed, data.rich_line);
            boost::hash_combine(seed, data.tags);
            return seed;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & id;
            ar & rich_line;
            ar & tags;
        }

    };

}

namespace std {

    template<typename RichLine,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    struct hash<map_matching_2::geometry::network::edge<RichLine, Container, Allocator>> {
        using type = map_matching_2::geometry::network::edge<RichLine, Container, Allocator>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_EDGE_HPP
