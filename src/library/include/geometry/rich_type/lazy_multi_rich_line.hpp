// Copyright (C) 2021-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_RICH_TYPE_LAZY_MULTI_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_TYPE_LAZY_MULTI_RICH_LINE_HPP

#include "multi_rich_line.hpp"
#include "multi_rich_line_data.hpp"

namespace map_matching_2::geometry {

    template<typename RichLine,
        template<typename, typename> typename Container,
        template<typename> typename Allocator> requires
        (util::is_random_access_v<Container<RichLine, Allocator<RichLine>>>)
    class eager_multi_rich_line;

    template<typename RichLine,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator> requires
        (util::is_random_access_v<Container<RichLine, Allocator<RichLine>>>)
    class lazy_multi_rich_line :
            public multi_rich_line_data<
                typename data<typename rich_line_traits<RichLine>::line_type>::length_type,
                typename data<typename rich_line_traits<RichLine>::line_type>::angle_type,
                Container, Allocator>,
            public multi_rich_line<RichLine, Container, Allocator> {

    public:
        using rich_line_type = RichLine;

        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using point_type = point_type_t<line_type>;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;

        using multi_rich_line_data_base_type = multi_rich_line_data<length_type, angle_type, Container, Allocator>;
        using multi_rich_line_base_type = multi_rich_line<rich_line_type, Container, Allocator>;

        using allocator_type = Allocator<void>;

        using rich_lines_container_type = Container<rich_line_type, Allocator<rich_line_type>>;

        using multi_line_type = typename models<point_type>::template multi_line_type<line_type>;

        constexpr lazy_multi_rich_line() requires
            (std::default_initializable<rich_line_type> and std::default_initializable<allocator_type>)
            : multi_rich_line_data_base_type{}, multi_rich_line_base_type{} {}

        constexpr explicit lazy_multi_rich_line(rich_line_type rich_line) requires
            (std::default_initializable<allocator_type>)
            : lazy_multi_rich_line{rich_lines_container_type{std::move(rich_line)}} {}

        explicit lazy_multi_rich_line(const multi_line_type &multi_line) requires
            (std::default_initializable<allocator_type>)
            : multi_rich_line_data_base_type{}, multi_rich_line_base_type{multi_line} {}

        lazy_multi_rich_line(const multi_line_type &multi_line, const allocator_type &allocator)
            : multi_rich_line_data_base_type{allocator}, multi_rich_line_base_type{multi_line, allocator} {}

        constexpr lazy_multi_rich_line(rich_line_type rich_line, const allocator_type &allocator)
            : lazy_multi_rich_line{
                    rich_lines_container_type{
                            std::initializer_list<rich_line_type>{std::move(rich_line)}, allocator
                    },
                    allocator
            } {}

        constexpr lazy_multi_rich_line(rich_lines_container_type rich_lines) requires
            (std::default_initializable<allocator_type>)
            : multi_rich_line_data_base_type{}, multi_rich_line_base_type{std::move(rich_lines)} {}

        constexpr lazy_multi_rich_line(rich_lines_container_type rich_lines,
                multi_rich_line_data_base_type data) requires
            (std::default_initializable<allocator_type>)
            : multi_rich_line_data_base_type{std::move(data)}, multi_rich_line_base_type{std::move(rich_lines)} {}

        constexpr lazy_multi_rich_line(rich_lines_container_type rich_lines, const allocator_type &allocator)
            : multi_rich_line_data_base_type{allocator},
            multi_rich_line_base_type{std::move(rich_lines), allocator} {}

        constexpr lazy_multi_rich_line(rich_lines_container_type rich_lines, multi_rich_line_data_base_type data,
                const allocator_type &allocator)
            : multi_rich_line_data_base_type{std::move(data), allocator},
            multi_rich_line_base_type{std::move(rich_lines), allocator} {}

        constexpr lazy_multi_rich_line(const lazy_multi_rich_line &other)
            : multi_rich_line_data_base_type{other}, multi_rich_line_base_type{other} {}

        template<typename RichLineTypeT,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr lazy_multi_rich_line(const multi_rich_line<RichLineTypeT, ContainerT, AllocatorT> &other)
            : multi_rich_line_data_base_type{}, multi_rich_line_base_type{other} {}

        template<typename RichLineTypeT,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr lazy_multi_rich_line(const eager_multi_rich_line<RichLineTypeT, ContainerT, AllocatorT> &other)
            : multi_rich_line_data_base_type{other}, multi_rich_line_base_type{other} {}

        constexpr lazy_multi_rich_line(lazy_multi_rich_line &&other) noexcept
            : multi_rich_line_data_base_type{std::move(other)}, multi_rich_line_base_type{std::move(other)} {}

        constexpr lazy_multi_rich_line &operator=(const lazy_multi_rich_line &other) {
            if (this != &other) {
                multi_rich_line_data_base_type::operator=(other);
                multi_rich_line_base_type::operator=(other);
            }
            return *this;
        }

        template<typename RichLineTypeT,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr lazy_multi_rich_line &operator=(const multi_rich_line<RichLineTypeT, ContainerT, AllocatorT> &other) {
            multi_rich_line_base_type::operator=(other);
            return *this;
        }

        template<typename RichLineTypeT,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr lazy_multi_rich_line &
        operator=(const eager_multi_rich_line<RichLineTypeT, ContainerT, AllocatorT> &other) {
            multi_rich_line_data_base_type::operator=(other);
            multi_rich_line_base_type::operator=(other);
            return *this;
        }

        constexpr lazy_multi_rich_line &operator=(lazy_multi_rich_line &&other) noexcept {
            if (this != &other) {
                multi_rich_line_data_base_type::operator=(std::move(other));
                multi_rich_line_base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr ~lazy_multi_rich_line() = default;

        [[nodiscard]] bool has_length() const {
            multi_rich_line_data_base_type::_compute_length(*this);
            return multi_rich_line_data_base_type::has_length();
        }

        [[nodiscard]] length_type length() const {
            multi_rich_line_data_base_type::_compute_length(*this);
            return multi_rich_line_data_base_type::length();
        }

        [[nodiscard]] bool has_azimuth() const {
            multi_rich_line_data_base_type::_compute_azimuth(*this);
            return multi_rich_line_data_base_type::has_azimuth();
        }

        [[nodiscard]] angle_type azimuth() const {
            multi_rich_line_data_base_type::_compute_azimuth(*this);
            return multi_rich_line_data_base_type::azimuth();
        }

        [[nodiscard]] bool has_directions() const {
            multi_rich_line_data_base_type::_compute_directions(*this);
            return multi_rich_line_data_base_type::has_directions();
        }

        [[nodiscard]] angle_type directions() const {
            multi_rich_line_data_base_type::_compute_directions(*this);
            return multi_rich_line_data_base_type::directions();
        }

        [[nodiscard]] angle_type absolute_directions() const {
            multi_rich_line_data_base_type::_compute_directions(*this);
            return multi_rich_line_data_base_type::absolute_directions();
        }

        [[nodiscard]] lazy_multi_rich_line sub_multi_rich_line(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type>) {
            return lazy_multi_rich_line{
                    multi_rich_line_base_type::sub_rich_line_container(from, to),
                    multi_rich_line_data_base_type::sub_data(from, to)
            };
        }

        [[nodiscard]] lazy_multi_rich_line sub_multi_rich_line(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            return lazy_multi_rich_line{
                    multi_rich_line_base_type::sub_rich_line_container(from, to, allocator),
                    multi_rich_line_data_base_type::sub_data(from, to, allocator),
                    allocator
            };
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_LAZY_MULTI_RICH_LINE_HPP
