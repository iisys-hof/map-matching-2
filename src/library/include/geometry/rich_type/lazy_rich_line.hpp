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

#ifndef MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_LINE_HPP

#include "types/geometry/rich_type/lazy_rich_segment.hpp"

#include "rich_line.hpp"
#include "rich_line_data.hpp"

#include "lazy_rich_segment.hpp"

namespace map_matching_2::geometry {

    template<typename Point,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    class eager_rich_line;

    template<typename Point,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    class lazy_rich_line
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
        using rich_segment_type = lazy_rich_segment_type<point_type>;
        using rich_referring_segment_type = lazy_rich_referring_segment_type<point_type>;

        using distance_type = typename data<line_type>::distance_type;
        using length_type = typename data<line_type>::length_type;
        using angle_type = typename data<line_type>::angle_type;

        using rich_line_data_base_type = rich_line_data<length_type, angle_type, Container, Allocator>;
        using rich_line_base_type = rich_line<point_type, Container, Allocator>;

        using allocator_type = Allocator<void>;

        using memory_rich_line_type = lazy_rich_line<point_type>;
        using memory_line_type = linestring<point_type>;

        constexpr lazy_rich_line() requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{} {}

        constexpr explicit lazy_rich_line(line_type line) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{std::move(line)} {}

        constexpr lazy_rich_line(line_type line, const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{std::move(line), allocator} {}

        constexpr lazy_rich_line(line_type line, rich_line_data_base_type data) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{std::move(data)},
            rich_line_base_type{std::move(line)} {}

        constexpr lazy_rich_line(line_type line, rich_line_data_base_type data, const allocator_type &allocator)
            : rich_line_data_base_type{std::move(data), allocator},
            rich_line_base_type{std::move(line), allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr explicit lazy_rich_line(const linestring<point_type, ContainerT, AllocatorT> &line) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{line} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const linestring<point_type, ContainerT, AllocatorT> &line,
                const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{line, allocator} {}

        constexpr lazy_rich_line(const lazy_rich_line &other)
            : rich_line_data_base_type{other}, rich_line_base_type{other} {}

        constexpr lazy_rich_line(const lazy_rich_line &other, const allocator_type &allocator)
            : rich_line_data_base_type{other, allocator}, rich_line_base_type{other, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const lazy_rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{other}, rich_line_base_type{other} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const lazy_rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{other, allocator}, rich_line_base_type{other, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{}, rich_line_base_type{other.line()} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{allocator}, rich_line_base_type{other.line(), allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const eager_rich_line<point_type, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : rich_line_data_base_type{other}, rich_line_base_type{other.line()} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line(const eager_rich_line<point_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : rich_line_data_base_type{other, allocator}, rich_line_base_type{other.line(), allocator} {}

        constexpr lazy_rich_line(lazy_rich_line &&other) noexcept
            : rich_line_data_base_type{std::move(other)}, rich_line_base_type{std::move(other)} {}

        constexpr lazy_rich_line(lazy_rich_line &&other, const allocator_type &allocator) noexcept
            : rich_line_data_base_type{std::move(other), allocator},
            rich_line_base_type{std::move(other), allocator} {}

        constexpr lazy_rich_line &operator=(const lazy_rich_line &other) {
            if (this != &other) {
                rich_line_data_base_type::operator=(other);
                rich_line_base_type::operator=(other);
            }
            return *this;
        }

        constexpr lazy_rich_line &operator=(line_type &&line) requires
            (std::default_initializable<allocator_type>) {
            rich_line_base_type::operator=(std::move(line));
            rich_line_data_base_type::_invalidate();
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line &operator=(const rich_line<point_type, ContainerT, AllocatorT> &other) {
            rich_line_base_type::line().assign(std::cbegin(other.line()), std::cend(other.line()));
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr lazy_rich_line &operator=(const eager_rich_line<point_type, ContainerT, AllocatorT> &other) {
            rich_line_data_base_type::operator=(other);
            rich_line_base_type::line().assign(std::cbegin(other.line()), std::cend(other.line()));
            return *this;
        }

        constexpr lazy_rich_line &operator=(lazy_rich_line &&other) noexcept {
            if (this != &other) {
                rich_line_data_base_type::operator=(std::move(other));
                rich_line_base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr ~lazy_rich_line() = default;

        [[nodiscard]] bool has_length() const {
            rich_line_data_base_type::_compute_length(*this);
            return rich_line_data_base_type::has_length();
        }

        [[nodiscard]] length_type length() const {
            rich_line_data_base_type::_compute_length(*this);
            return rich_line_data_base_type::length();
        }

        [[nodiscard]] bool has_azimuth() const {
            rich_line_data_base_type::_compute_azimuth(*this);
            return rich_line_data_base_type::has_azimuth();
        }

        [[nodiscard]] angle_type azimuth() const {
            rich_line_data_base_type::_compute_azimuth(*this);
            return rich_line_data_base_type::azimuth();
        }

        [[nodiscard]] bool has_directions() const {
            rich_line_data_base_type::_compute_directions(*this);
            return rich_line_data_base_type::has_directions();
        }

        [[nodiscard]] angle_type directions() const {
            rich_line_data_base_type::_compute_directions(*this);
            return rich_line_data_base_type::directions();
        }

        [[nodiscard]] angle_type absolute_directions() const {
            rich_line_data_base_type::_compute_directions(*this);
            return rich_line_data_base_type::absolute_directions();
        }

        [[nodiscard]] bool has_length(std::size_t pos) const {
            rich_line_data_base_type::_compute_lengths(*this);
            rich_line_data_base_type::_initialize_length(*this, pos);
            return rich_line_data_base_type::has_length(pos);
        }

        [[nodiscard]] length_type length(std::size_t pos) const {
            rich_line_data_base_type::_compute_lengths(*this);
            rich_line_data_base_type::_initialize_length(*this, pos);
            return rich_line_data_base_type::length(pos);
        }

        [[nodiscard]] bool has_azimuth(std::size_t pos) const {
            rich_line_data_base_type::_compute_azimuths(*this);
            rich_line_data_base_type::_initialize_azimuth(*this, pos);
            return rich_line_data_base_type::has_azimuth(pos);
        }

        [[nodiscard]] angle_type azimuth(std::size_t pos) const {
            rich_line_data_base_type::_compute_azimuths(*this);
            rich_line_data_base_type::_initialize_azimuth(*this, pos);
            return rich_line_data_base_type::azimuth(pos);
        }

        [[nodiscard]] bool has_direction(std::size_t pos) const {
            rich_line_data_base_type::_compute_azimuths(*this);
            rich_line_data_base_type::_initialize_direction(*this, pos);
            return rich_line_data_base_type::has_direction(pos);
        }

        [[nodiscard]] angle_type direction(std::size_t pos) const {
            rich_line_data_base_type::_compute_azimuths(*this);
            rich_line_data_base_type::_initialize_direction(*this, pos);
            return rich_line_data_base_type::direction(pos);
        }

        [[nodiscard]] angle_type absolute_direction(std::size_t pos) const {
            rich_line_data_base_type::_compute_azimuths(*this);
            rich_line_data_base_type::_initialize_direction(*this, pos);
            return rich_line_data_base_type::absolute_direction(pos);
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(std::size_t pos) const {
            return rich_segment_type{
                    this->segment(pos),
                    pos < this->_lengths.size() ? length(pos) : not_initialized<length_type>,
                    pos < this->_azimuths.size() ? azimuth(pos) : not_initialized<angle_type>
            };
        }

        [[nodiscard]] constexpr rich_segment_type rich_segment(typename line_type::const_iterator it) const {
            return rich_segment(std::distance(std::cbegin(this->_line), it));
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(std::size_t pos) const {
            return rich_referring_segment_type{
                    this->referring_segment(pos),
                    pos < this->_lengths.size() ? length(pos) : not_initialized<length_type>,
                    pos < this->_azimuths.size() ? azimuth(pos) : not_initialized<angle_type>
            };
        }

        [[nodiscard]] constexpr rich_referring_segment_type rich_referring_segment(
                typename line_type::const_iterator it) const {
            return rich_referring_segment(std::distance(std::cbegin(this->_line), it));
        }

        [[nodiscard]] lazy_rich_line sub_rich_line(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type>) {
            return lazy_rich_line{
                    rich_line_base_type::sub_line(from, to),
                    rich_line_data_base_type::sub_data(from, to)
            };
        }

        [[nodiscard]] lazy_rich_line sub_rich_line(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            return lazy_rich_line{
                    rich_line_base_type::sub_line(from, to, allocator),
                    rich_line_data_base_type::sub_data(from, to, allocator),
                    allocator
            };
        }

        void erase(std::size_t index, bool invalidate) {
            rich_line_base_type::erase(index);
            rich_line_data_base_type::_erase_and_replace(*this, index);
            if (invalidate) {
                bool azimuth = index == 0 or index + 1 == this->size();
                rich_line_data_base_type::_invalidate(true, azimuth, true, false, false);
            }
        }

        void erase(const std::vector<std::size_t> &indices) {
            if (not indices.empty()) {
                bool azimuth = indices.front() == 0 or indices.back() + 1 == this->size();
                for (std::size_t i = indices.size(); i-- > 0;) {
                    erase(indices[i], false);
                }
                rich_line_data_base_type::_invalidate(true, azimuth, true, false, false);
            }
        }

        template<is_rich_line... RichLines>
        lazy_rich_line &merge(RichLines &&... lazy_rich_lines) requires
            (... && std::is_base_of_v<lazy_rich_line, RichLines>) {
            rich_line_data_base_type::template _merge<RichLines...>(*this, std::move(lazy_rich_lines)...);
            rich_line_base_type::template merge<RichLines...>(std::move(lazy_rich_lines)...);
            rich_line_data_base_type::_invalidate(true, true, true, false, false);

            return *this;
        }

        template<is_rich_line RichLine>
        lazy_rich_line &merge(std::vector<RichLine> &&lazy_rich_lines) requires
            (std::is_base_of_v<lazy_rich_line, RichLine>) {
            using _rich_line_type = RichLine;
            rich_line_data_base_type::template _merge<_rich_line_type>(*this, std::move(lazy_rich_lines));
            rich_line_base_type::template merge<_rich_line_type>(std::move(lazy_rich_lines));
            rich_line_data_base_type::_invalidate(true, true, true, false, false);

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

#endif //MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_LINE_HPP
