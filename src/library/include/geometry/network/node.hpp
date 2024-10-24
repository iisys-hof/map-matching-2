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

#include <cstdint>
#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "io/helper/tag_helper.hpp"

#include "geometry/algorithm/reproject.hpp"
#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/wkt.hpp"

namespace map_matching_2::geometry::network {

    template<typename Point>
    struct node {

        using point_type = Point;

        std::uint64_t id{};
        point_type point;

        constexpr node() requires
            (std::default_initializable<point_type>)
            : point{} {}

        constexpr node(const std::uint64_t id, point_type point)
            : id{id}, point{std::move(point)} {}

        template<typename InPoint>
        node(const std::uint64_t id, InPoint input_point,
                const geometry::point_reprojector_variant &reprojector_variant) requires
            (geometry::is_point<InPoint>)
            : id{id} {
            reproject_point(input_point, point, reprojector_variant);
        }

        constexpr node(const node &other)
            : id{other.id}, point{other.point} {}

        constexpr node(node &&other) noexcept
            : id{std::move(other.id)}, point{std::move(other.point)} {}

        constexpr node &operator=(const node &other) {
            if (this != &other) {
                id = other.id;
                point = other.point;
            }
            return *this;
        }

        constexpr node &operator=(node &&other) noexcept {
            if (this != &other) {
                id = std::move(other.id);
                point = std::move(other.point);
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
            const auto tags = tag_helper.node_tags(id);
            if (not tags.empty()) {
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
            return seed;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & id;
            ar & point;
        }

    };

}

namespace std {

    template<typename Point>
    struct hash<map_matching_2::geometry::network::node<Point>> {
        using type = map_matching_2::geometry::network::node<Point>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_NODE_HPP
