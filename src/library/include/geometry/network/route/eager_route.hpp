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

#ifndef MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP
#define MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP

#include "route.hpp"
#include "route_data.hpp"

namespace map_matching_2::geometry::network {

    template<typename RichLineVariant>
    class eager_route
            : public route_data<
                typename rich_line_traits<std::variant_alternative_t<0, RichLineVariant>>::length_type,
                typename rich_line_traits<std::variant_alternative_t<0, RichLineVariant>>::angle_type>,
            public route<RichLineVariant> {

    public:
        using rich_line_variant_type = RichLineVariant;
        using rich_line_reference_variant_type = util::variant_wrapper_t<
            std::reference_wrapper, rich_line_variant_type, true>;

        template<is_rich_line RichLine>
        using rich_line_reference_type = std::reference_wrapper<RichLine>;

        template<is_rich_line RichLine>
        using rich_line_const_reference_type = std::reference_wrapper<const RichLine>;

        using rich_line_alternative_type = std::variant_alternative_t<0, rich_line_variant_type>;
        // using line_type = typename rich_line_traits<rich_line_type>::line_type;
        // using point_type = typename rich_line_traits<rich_line_type>::point_type;
        // using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        // using segment_type = typename rich_line_traits<rich_line_alternative_type>::segment_type;
        using length_type = typename rich_line_traits<rich_line_alternative_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_alternative_type>::angle_type;

        using route_data_base_type = route_data<length_type, angle_type>;
        using route_base_type = route<rich_line_variant_type>;

        constexpr eager_route() = default;

        constexpr explicit eager_route(bool is_invalid)
            : route_data_base_type{}, route_base_type{is_invalid} {}

        template<is_rich_line RichLine>
        explicit eager_route(RichLine rich_line) requires
            (util::variant_alternative_t<rich_line_variant_type, RichLine>)
            : eager_route{std::list<rich_line_variant_type>{std::move(rich_line)}} {}

        explicit eager_route(std::list<rich_line_variant_type> own_rich_lines)
            : route_data_base_type{}, route_base_type{std::move(own_rich_lines)} {
            route_data_base_type::_compute(*this);
        }

        template<is_rich_line RichLine>
        explicit eager_route(rich_line_const_reference_type<RichLine> rich_line_reference) requires
            (util::variant_alternative_t<rich_line_reference_variant_type, rich_line_const_reference_type<RichLine>>)
            : eager_route{std::vector<rich_line_reference_variant_type>{std::move(rich_line_reference)}} {}

        explicit eager_route(std::vector<rich_line_reference_variant_type> rich_lines_references)
            : route_data_base_type{}, route_base_type{std::move(rich_lines_references)} {
            route_data_base_type::_compute(*this);
        }

        eager_route(std::list<rich_line_variant_type> rich_lines,
                std::vector<rich_line_reference_variant_type> rich_lines_references)
            : route_data_base_type{}, route_base_type{std::move(rich_lines), std::move(rich_lines_references)} {
            route_data_base_type::_compute(*this);
        }

        eager_route(const eager_route &other)
            : route_data_base_type{other}, route_base_type{other} {}

        eager_route(eager_route &&other) noexcept
            : route_data_base_type{std::move(other)}, route_base_type{std::move(other)} {}

        eager_route &operator=(const eager_route &other) {
            if (this != &other) {
                route_data_base_type::operator=(other);
                route_base_type::operator=(other);
            }
            return *this;
        }

        eager_route &operator=(eager_route &&other) noexcept {
            if (this != &other) {
                route_data_base_type::operator=(std::move(other));
                route_base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr ~eager_route() = default;

        [[nodiscard]] constexpr bool is_connected() const {
            return route_data_base_type::is_connected();
        }

        [[nodiscard]] constexpr bool has_length() const {
            return route_data_base_type::has_length();
        }

        [[nodiscard]] constexpr length_type length() const {
            return route_data_base_type::length();
        }

        [[nodiscard]] constexpr bool has_azimuth() const {
            return route_data_base_type::has_azimuth();
        }

        [[nodiscard]] constexpr angle_type azimuth() const {
            return route_data_base_type::azimuth();
        }

        [[nodiscard]] constexpr bool has_directions() const {
            return route_data_base_type::has_directions();
        }

        [[nodiscard]] constexpr angle_type directions() const {
            return route_data_base_type::directions();
        }

        [[nodiscard]] constexpr angle_type absolute_directions() const {
            return route_data_base_type::absolute_directions();
        }

        template<is_route RouteT = eager_route>
        [[nodiscard]] RouteT sub_route(std::size_t from, std::size_t to) const {
            return route_base_type::template sub_route<RouteT>(from, to);
        }

        template<is_rich_line RichLineT, is_route RouteT = eager_route>
        [[nodiscard]] RouteT sub_route(std::size_t from, std::size_t from_pos,
                std::size_t to, std::size_t to_pos) const {
            return route_base_type::template sub_route<RichLineT, RouteT>(from, from_pos, to, to_pos);
        }

        template<is_rich_line RichLineT, is_route RouteT = eager_route>
        [[nodiscard]] RouteT trim_merge(const RouteT &other) const {
            return route_base_type::template trim_merge<RichLineT, RouteT>(other);
        }

        template<is_rich_line RichLineT, is_route RouteT = eager_route>
        [[nodiscard]] static RouteT merge(
                std::vector<RouteT> &&routes, bool connect = false, bool non_connect_except = false) {
            return route_base_type::template merge<RichLineT, RouteT>(std::move(routes), connect, non_connect_except);
        }

        template<is_rich_line RichLineT, is_route RouteT = eager_route>
        static void join(RouteT &a, RouteT &b, bool retain_reversal = false) {
            return route_base_type::template join<RichLineT, RouteT>(a, b, retain_reversal);
        }

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_NETWORK_ROUTE_EAGER_ROUTE_HPP
