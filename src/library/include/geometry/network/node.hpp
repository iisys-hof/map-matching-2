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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_NODE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_NODE_HPP

#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <osmium/osm/types.hpp>

#include "io/helper/tag_helper.hpp"

#include "geometry/algorithm/reproject.hpp"
#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/wkt.hpp"

namespace map_matching_2::geometry::network {

    template<typename Point,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    struct node {

        using point_type = Point;
        using tag_type = std::uint64_t;

        using allocator_type = Allocator<void>;

        using tags_container_type = Container<tag_type, Allocator<tag_type>>;

        osmium::object_id_type id{};
        point_type point;
        tags_container_type tags;

        constexpr node() requires
            (std::default_initializable<point_type> and std::default_initializable<tags_container_type> and
                std::default_initializable<allocator_type>)
            : point{}, tags{} {}

        constexpr node(osmium::object_id_type id, point_type point) requires
            (std::default_initializable<tags_container_type> and std::default_initializable<allocator_type>)
            : node{id, std::move(point), tags_container_type{}} {}

        constexpr node(osmium::object_id_type id, point_type point, tags_container_type tags)
            : id{id}, point{std::move(point)}, tags{std::move(tags)} {}

        template<typename InPoint>
        node(osmium::object_id_type id, InPoint input_point,
                const geometry::point_reprojector_variant &reprojector_variant) requires
            (std::default_initializable<tags_container_type> and std::default_initializable<allocator_type> and
                geometry::is_point<InPoint>)
            : node{std::move(id), std::move(input_point), reprojector_variant, tags_container_type{}} {}

        template<typename InPoint>
        node(osmium::object_id_type id, InPoint input_point,
                const geometry::point_reprojector_variant &reprojector_variant, tags_container_type tags) requires
            (geometry::is_point<InPoint>)
            : id{std::move(id)}, tags{std::move(tags)} {
            reproject_point(input_point, point, reprojector_variant);
        }

        constexpr node(const node &other)
            : id{other.id}, point{other.point}, tags{other.tags} {}

        constexpr node(const node &other, const allocator_type &allocator)
            : id{other.id}, point{other.point}, tags{other.tags, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr node(const node<point_type, ContainerT, AllocatorT> &other)
            : id{other.id}, point{other.point}, tags{std::cbegin(other.tags), std::cend(other.tags)} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr node(const node<point_type, ContainerT, AllocatorT> &other, const allocator_type &allocator)
            : id{other.id}, point{other.point}, tags{std::cbegin(other.tags), std::cend(other.tags), allocator} {}

        constexpr node(node &&other) noexcept
            : id{std::move(other.id)}, point{std::move(other.point)}, tags{std::move(other.tags)} {}

        constexpr node(node &&other, const allocator_type &allocator) noexcept
            : id{std::move(other.id)}, point{std::move(other.point)}, tags{std::move(other.tags), allocator} {}

        constexpr node &operator=(const node &other) {
            if (this != &other) {
                id = other.id;
                point = other.point;
                tags = other.tags;
            }
            return *this;
        }

        constexpr node &operator=(node &&other) noexcept {
            if (this != &other) {
                id = std::move(other.id);
                point = std::move(other.point);
                tags = std::move(other.tags);
            }
            return *this;
        }

        constexpr ~node() = default;

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(point);
        }

        [[nodiscard]] static std::vector<std::string> header() {
            return {"id", "geometry", "tags"};
        }

        template<typename TagHelper>
        [[nodiscard]] std::vector<std::string> row(const TagHelper &tag_helper) const {
            std::vector<std::string> row;
            row.reserve(3);
            row.emplace_back(std::to_string(id));
            row.emplace_back(wkt());
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
            std::string str = "Node(";
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

        friend constexpr bool operator==(const node &left, const node &right) {
            return left.id == right.id and geometry::equals_points(left.point, right.point);
        }

        friend constexpr bool operator!=(const node &left, const node &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const node &node) {
            return out << node.str();
        }

        friend std::size_t hash_value(const node &data) {
            std::size_t seed = 0;
            boost::hash_combine(seed, data.id);
            boost::hash_combine(seed, data.point);
            boost::hash_combine(seed, data.tags);
            return seed;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & id;
            ar & point;
            ar & tags;
        }

    };

}

namespace std {

    template<typename Point,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    struct hash<map_matching_2::geometry::network::node<Point, Container, Allocator>> {
        using type = map_matching_2::geometry::network::node<Point, Container, Allocator>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_NODE_HPP
