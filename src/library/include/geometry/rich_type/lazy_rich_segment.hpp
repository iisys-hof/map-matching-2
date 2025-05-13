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

#ifndef MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_SEGMENT_HPP

#include "rich_segment.hpp"
#include "rich_segment_data.hpp"

namespace map_matching_2::geometry {

    template<typename Segment>
    class eager_rich_segment;

    template<typename Segment>
    class lazy_rich_segment :
            public rich_segment_data<
                typename data<Segment>::length_type,
                typename data<Segment>::angle_type>,
            public rich_segment<Segment> {

    public:
        using segment_type = Segment;
        using distance_type = typename data<segment_type>::distance_type;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;
        using rich_segment_data_base_type = rich_segment_data<length_type, angle_type>;
        using rich_segment_base_type = rich_segment<Segment>;
        using point_type = point_type_t<segment_type>;

        constexpr lazy_rich_segment() requires
            (std::default_initializable<segment_type>)
            : rich_segment_data_base_type{}, rich_segment_base_type{} {}

        constexpr lazy_rich_segment(point_type a, point_type b) requires
            (is_segment<segment_type>)
            : rich_segment_data_base_type{}, rich_segment_base_type{std::move(a), std::move(b)} {}

        constexpr lazy_rich_segment(const point_type &a, const point_type &b) requires
            (is_referring_segment<segment_type> or is_pointing_segment<segment_type>)
            : rich_segment_data_base_type{}, rich_segment_base_type{a, b} {}

        constexpr explicit lazy_rich_segment(segment_type segment)
            : rich_segment_data_base_type{}, rich_segment_base_type{std::move(segment)} {}

        constexpr lazy_rich_segment(segment_type segment, length_type length, angle_type azimuth)
            : rich_segment_data_base_type{std::move(length), std::move(azimuth)},
            rich_segment_base_type{std::move(segment)} {}

        constexpr ~lazy_rich_segment() = default;

        constexpr lazy_rich_segment(const lazy_rich_segment &other) requires
            (std::copy_constructible<segment_type>)
            : rich_segment_data_base_type{other}, rich_segment_base_type{other} {}

        template<typename SegmentT>
        constexpr lazy_rich_segment(const lazy_rich_segment<SegmentT> &other) requires
            (!std::same_as<SegmentT, segment_type> or !std::copy_constructible<segment_type>)
            : rich_segment_data_base_type{other}, rich_segment_base_type{other} {}

        template<typename SegmentT>
        constexpr lazy_rich_segment(const rich_segment<SegmentT> &other)
            : rich_segment_data_base_type{}, rich_segment_base_type{other} {}

        template<typename SegmentT>
        constexpr lazy_rich_segment(const eager_rich_segment<SegmentT> &other)
            : rich_segment_data_base_type{other}, rich_segment_base_type{other} {}

        constexpr lazy_rich_segment(lazy_rich_segment &&other) noexcept requires
            (std::move_constructible<segment_type>)
            : rich_segment_data_base_type{std::move(other)}, rich_segment_base_type{std::move(other)} {}

        template<typename SegmentT>
        constexpr lazy_rich_segment(lazy_rich_segment<SegmentT> &&other) noexcept requires
            (!std::same_as<SegmentT, segment_type> or !std::move_constructible<segment_type>)
            : rich_segment_data_base_type{std::move(other)}, rich_segment_base_type{std::move(other)} {}

        constexpr lazy_rich_segment &operator=(const lazy_rich_segment &other) requires
            (std::copyable<segment_type>) {
            if (this != &other) {
                rich_segment_data_base_type::operator=(other);
                rich_segment_base_type::operator=(other);
            }
            return *this;
        }

        template<typename SegmentT>
        constexpr lazy_rich_segment &operator=(const lazy_rich_segment<SegmentT> &other) requires
            (!std::same_as<SegmentT, segment_type> or !std::copyable<segment_type>) {
            rich_segment_data_base_type::operator=(other);
            rich_segment_base_type::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        constexpr lazy_rich_segment &operator=(const rich_segment<SegmentT> &other) {
            rich_segment_base_type::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        constexpr lazy_rich_segment &operator=(const eager_rich_segment<SegmentT> &other) {
            rich_segment_data_base_type::operator=(other);
            rich_segment_base_type::operator=(other);
            return *this;
        }

        constexpr lazy_rich_segment &operator=(lazy_rich_segment &&other) noexcept requires
            (std::movable<segment_type>) {
            if (this != &other) {
                rich_segment_data_base_type::operator=(std::move(other));
                rich_segment_base_type::operator=(std::move(other));
            }
            return *this;
        }

        template<typename SegmentT>
        constexpr lazy_rich_segment &operator=(lazy_rich_segment<SegmentT> &&other) noexcept requires
            (!std::same_as<SegmentT, segment_type> or !std::movable<segment_type>) {
            rich_segment_data_base_type::operator=(std::move(other));
            rich_segment_base_type::operator=(std::move(other));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            rich_segment_data_base_type::_compute_length(*this);
            return rich_segment_data_base_type::has_length();
        }

        [[nodiscard]] length_type length() const {
            rich_segment_data_base_type::_compute_length(*this);
            return rich_segment_data_base_type::length();
        }

        [[nodiscard]] bool has_azimuth() const {
            rich_segment_data_base_type::_compute_azimuth(*this);
            return rich_segment_data_base_type::has_azimuth();
        }

        [[nodiscard]] angle_type azimuth() const {
            rich_segment_data_base_type::_compute_azimuth(*this);
            return rich_segment_data_base_type::azimuth();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (std::default_initializable<segment_type>) {
            ar & boost::serialization::base_object<rich_segment_data_base_type>(*this);
            ar & boost::serialization::base_object<rich_segment_base_type>(*this);
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_LAZY_RICH_SEGMENT_HPP
