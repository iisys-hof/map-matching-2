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

#ifndef MAP_MATCHING_2_RICH_TYPE_EAGER_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_TYPE_EAGER_RICH_LINE_HPP

#include "types/geometry/rich_type/eager_rich_segment.hpp"

#include "rich_line.hpp"
#include "rich_line_data.hpp"

#include "eager_rich_segment.hpp"

namespace map_matching_2::geometry {

    template<typename Point,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    class lazy_rich_line;

    template<typename Point,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    class eager_rich_line
            : public rich_line_data<
                typename data<Point>::length_type,
                typename data<Point>::angle_type,
                Container, Allocator>,
            public rich_line<Point, Container, Allocator> {

    public:
        using point_type = Point;
        using line_type = linestring<Point, Container, Allocator>;

        using segment_type = typename models<point_type>::segment_type;
        using referring_segment_type = typename models<point_type>::referring_segment_type;
        using coordinate_system_type = typename data<point_type>::coordinate_system_type;
        using rich_segment_type = eager_rich_segment_type<point_type>;
        using rich_referring_segment_type = eager_rich_referring_segment_type<point_type>;

        using distance_type = typename data<line_type>::distance_type;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;

        using rich_line_data_base_type = rich_line_data<length_type, angle_type, Container, Allocator>;
        using rich_line_base_type = rich_line<point_type, Container, Allocator>;

        using allocator_type = Allocator<void>;

        using memory_rich_line_type = eager_rich_line<point_type>;
        using memory_line_type = linestring<point_type>;

        constexpr eager_rich_line() requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{} {}

        explicit eager_rich_line(line_type line) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{std::move(line)} {
            rich_line_data_base_type::_compute(*this);
        }

        eager_rich_line(line_type line, const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{std::move(line), allocator} {
            rich_line_data_base_type::_compute(*this);
        }

        eager_rich_line(line_type line, rich_line_data_base_type data) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{std::move(data)},
            rich_line_base_type{std::move(line)} {
            rich_line_data_base_type::_compute(*this);
        }

        eager_rich_line(line_type line, rich_line_data_base_type data, const allocator_type &allocator)
            : rich_line_data_base_type{std::move(data), allocator},
            rich_line_base_type{std::move(line), allocator} {
            rich_line_data_base_type::_compute(*this);
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        explicit eager_rich_line(const linestring<point_type, ContainerT, AllocatorT> &line) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{line} {
            rich_line_data_base_type::_compute(*this);
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line(const linestring<point_type, ContainerT, AllocatorT> &line,
                const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{line, allocator} {
            rich_line_data_base_type::_compute(*this);
        }

        constexpr eager_rich_line(const eager_rich_line &other)
            : rich_line_data_base_type{other}, rich_line_base_type{other} {}

        constexpr eager_rich_line(const eager_rich_line &other, const allocator_type &allocator)
            : rich_line_data_base_type{other, allocator}, rich_line_base_type{other, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr eager_rich_line(const eager_rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{other}, rich_line_base_type{other} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr eager_rich_line(const eager_rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{other, allocator}, rich_line_base_type{other, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{other.line()} {
            rich_line_data_base_type::_compute(*this);
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{other.line(), allocator} {
            rich_line_data_base_type::_compute(*this);
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line(const lazy_rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{
                    other.length(), other.azimuth(), other.directions(), other.absolute_directions()
            }, rich_line_base_type{other.line()} {
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()),
                    std::back_inserter(rich_line_data_base_type::_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()),
                    std::back_inserter(rich_line_data_base_type::_azimuths));
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line(const lazy_rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{
                    other.length(), other.azimuth(), other.directions(), other.absolute_directions(), allocator
            }, rich_line_base_type{other.line(), allocator} {
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()),
                    std::back_inserter(rich_line_data_base_type::_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()),
                    std::back_inserter(rich_line_data_base_type::_azimuths));
        }

        constexpr eager_rich_line(eager_rich_line &&other) noexcept
            : rich_line_data_base_type{std::move(other)}, rich_line_base_type{std::move(other)} {}

        constexpr eager_rich_line(eager_rich_line &&other, const allocator_type &allocator) noexcept
            : rich_line_data_base_type{std::move(other), allocator},
            rich_line_base_type{std::move(other), allocator} {}

        constexpr eager_rich_line &operator=(const eager_rich_line &other) {
            if (this != &other) {
                rich_line_data_base_type::operator=(other);
                rich_line_base_type::operator=(other);
            }
            return *this;
        }

        constexpr eager_rich_line &operator=(line_type &&line) requires
            (std::default_initializable<allocator_type>) {
            rich_line_base_type::operator=(std::move(line));
            rich_line_data_base_type::_invalidate();
            rich_line_data_base_type::_compute(*this);
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line &operator=(const rich_line<point_type, ContainerT, AllocatorT> &other) {
            rich_line_base_type::line().assign(std::cbegin(other.line()), std::cend(other.line()));
            rich_line_data_base_type::_compute(*this);
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        eager_rich_line &operator=(const lazy_rich_line<point_type, ContainerT, AllocatorT> &other) {
            rich_line_data_base_type::_length = other.length();
            rich_line_data_base_type::_azimuth = other.azimuth();
            rich_line_data_base_type::_directions = other.directions();
            rich_line_data_base_type::_absolute_directions = other.absolute_directions();
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()),
                    std::back_inserter(rich_line_data_base_type::_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()),
                    std::back_inserter(rich_line_data_base_type::_azimuths));
            rich_line_base_type::line().assign(std::cbegin(other.line()), std::cend(other.line()));
            return *this;
        }

        constexpr eager_rich_line &operator=(eager_rich_line &&other) noexcept {
            if (this != &other) {
                rich_line_data_base_type::operator=(std::move(other));
                rich_line_base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr ~eager_rich_line() = default;

        [[nodiscard]] constexpr bool has_length() const {
            return rich_line_data_base_type::has_length();
        }

        [[nodiscard]] constexpr length_type length() const {
            return rich_line_data_base_type::length();
        }

        [[nodiscard]] constexpr bool has_azimuth() const {
            return rich_line_data_base_type::has_azimuth();
        }

        [[nodiscard]] constexpr angle_type azimuth() const {
            return rich_line_data_base_type::azimuth();
        }

        [[nodiscard]] constexpr bool has_directions() const {
            return rich_line_data_base_type::has_directions();
        }

        [[nodiscard]] constexpr angle_type directions() const {
            return rich_line_data_base_type::directions();
        }

        [[nodiscard]] constexpr angle_type absolute_directions() const {
            return rich_line_data_base_type::absolute_directions();
        }

        [[nodiscard]] constexpr bool has_length(std::size_t pos) const {
            return rich_line_data_base_type::has_length(pos);
        }

        [[nodiscard]] constexpr length_type length(std::size_t pos) const {
            return rich_line_data_base_type::length(pos);
        }

        [[nodiscard]] constexpr bool has_azimuth(std::size_t pos) const {
            return rich_line_data_base_type::has_azimuth(pos);
        }

        [[nodiscard]] constexpr angle_type azimuth(std::size_t pos) const {
            return rich_line_data_base_type::azimuth(pos);
        }

        [[nodiscard]] constexpr bool has_direction(std::size_t pos) const {
            return rich_line_data_base_type::has_direction(pos);
        }

        [[nodiscard]] constexpr angle_type direction(std::size_t pos) const {
            return rich_line_data_base_type::direction(pos);
        }

        [[nodiscard]] constexpr angle_type absolute_direction(std::size_t pos) const {
            return rich_line_data_base_type::absolute_direction(pos);
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(std::size_t pos) const {
            return rich_segment_type{this->segment(pos), length(pos), azimuth(pos)};
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(typename line_type::const_iterator it) const {
            return rich_segment(std::distance(std::cbegin(this->_line), it));
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(std::size_t pos) const {
            return rich_referring_segment_type{this->referring_segment(pos), length(pos), azimuth(pos)};
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(
                typename line_type::const_iterator it) const {
            return rich_referring_segment(std::distance(std::cbegin(this->_line), it));
        }

        [[nodiscard]] eager_rich_line sub_rich_line(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type>) {
            return eager_rich_line{
                    rich_line_base_type::sub_line(from, to),
                    rich_line_data_base_type::sub_data(from, to)
            };
        }

        [[nodiscard]] eager_rich_line sub_rich_line(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            return eager_rich_line{
                    rich_line_base_type::sub_line(from, to, allocator),
                    rich_line_data_base_type::sub_data(from, to, allocator),
                    allocator
            };
        }

        void erase(std::size_t index, bool rebuild) {
            rich_line_base_type::erase(index);
            rich_line_data_base_type::_erase_and_replace(*this, index);
            if (rebuild) {
                bool azimuth = index == 0 or index + 1 == this->size();
                rich_line_data_base_type::_rebuild(*this, true, azimuth, true);
            }
        }

        void erase(const std::vector<std::size_t> &indices) {
            if (not indices.empty()) {
                bool azimuth = indices.front() == 0 or indices.back() + 1 == this->size();
                for (std::size_t i = indices.size(); i-- > 0;) {
                    erase(indices[i], false);
                }
                rich_line_data_base_type::_rebuild(*this, true, azimuth, true);
            }
        }

        template<is_rich_line... RichLines>
        eager_rich_line &merge(RichLines &&... eager_rich_lines) requires
            (... && std::is_base_of_v<eager_rich_line, RichLines>) {
            rich_line_data_base_type::template _merge<RichLines...>(*this, std::move(eager_rich_lines)...);
            rich_line_base_type::template merge<RichLines...>(std::move(eager_rich_lines)...);
            rich_line_data_base_type::_rebuild(*this, true, true, true);

            return *this;
        }

        template<is_rich_line RichLine>
        eager_rich_line &merge(std::vector<RichLine> &&eager_rich_lines) requires
            (std::is_base_of_v<eager_rich_line, RichLine>) {
            using _rich_line_type = RichLine;
            rich_line_data_base_type::template _merge<_rich_line_type>(*this, std::move(eager_rich_lines));
            rich_line_base_type::template merge<_rich_line_type>(std::move(eager_rich_lines));
            rich_line_data_base_type::_rebuild(*this, true, true, true);

            return *this;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>) {
            ar & boost::serialization::base_object<rich_line_data_base_type>(*this);
            ar & boost::serialization::base_object<rich_line_base_type>(*this);
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_EAGER_RICH_LINE_HPP
