// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_HPP

#include <format>

#include <boost/serialization/serialization.hpp>

#include "geometry/point.hpp"
#include "geometry/concepts.hpp"
#include "geometry/traits.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/length.hpp"
#include "geometry/algorithm/azimuth.hpp"
#include "geometry/algorithm/wkt.hpp"

#include "exception.hpp"

namespace map_matching_2::geometry {

    template<typename Segment>
    class rich_segment {

    public:
        using segment_type = Segment;
        using point_type = point_type_t<segment_type>;
        using distance_type = typename data<segment_type>::distance_type;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;

        constexpr rich_segment() requires
            (std::default_initializable<segment_type>)
            : _segment{} {}

        constexpr rich_segment(point_type a, point_type b) requires
            (is_segment<segment_type>)
            : rich_segment{segment_type{std::move(a), std::move(b)}} {}

        constexpr rich_segment(const point_type &a, const point_type &b) requires
            (is_referring_segment<segment_type> or is_pointing_segment<segment_type>)
            : rich_segment{segment_type{a, b}} {}

        constexpr explicit rich_segment(segment_type segment)
            : _segment{std::move(segment)} {}

        constexpr rich_segment(const rich_segment &other) requires
            (std::copy_constructible<segment_type>)
            : _segment{other._segment} {}

        template<typename SegmentT>
        constexpr rich_segment(const rich_segment<SegmentT> &other) requires
            (!std::same_as<SegmentT, segment_type> or !std::copy_constructible<segment_type>)
            : _segment{segment_type{other.first(), other.second()}} {}

        constexpr rich_segment(rich_segment &&other) noexcept requires
            (std::move_constructible<segment_type>)
            : _segment{std::move(other._segment)} {}

        template<typename SegmentT>
        constexpr rich_segment(rich_segment<SegmentT> &&other) noexcept requires
            (!std::same_as<SegmentT, segment_type> or !std::move_constructible<segment_type>)
            : _segment{segment_type{other.first(), other.second()}} {}

        constexpr rich_segment &operator=(const rich_segment &other) requires
            (std::copyable<segment_type>) {
            if (this != &other) {
                _segment = other._segment;
            }
            return *this;
        }

        template<typename SegmentT>
        constexpr rich_segment &operator=(const rich_segment<SegmentT> &other) requires
            (!std::same_as<SegmentT, segment_type> or !std::copyable<segment_type>) {
            first() = other.first();
            second() = other.second();
            return *this;
        }

        constexpr rich_segment &operator=(rich_segment &&other) noexcept requires
            (std::movable<segment_type>) {
            if (this != &other) {
                _segment = std::move(other._segment);
            }
            return *this;
        }

        template<typename SegmentT>
        constexpr rich_segment &operator=(rich_segment<SegmentT> &&other) noexcept requires
            (!std::same_as<SegmentT, segment_type> or !std::movable<segment_type>) {
            first() = std::move(other.first());
            second() = std::move(other.second());
            return *this;
        }

        constexpr ~rich_segment() = default;

        [[nodiscard]] constexpr bool equals_points() const {
            return geometry::equals_points(this->first(), this->second());
        }

        [[nodiscard]] constexpr bool has_length() const {
            return not equals_points();
        }

        [[nodiscard]] length_type length() const {
            if (has_length()) {
                return geometry::length_segment(_segment);
            }

            throw length_computation_exception{std::format("cannot compute length for segment: {}", wkt())};
        }

        [[nodiscard]] constexpr bool has_azimuth() const {
            return not equals_points();
        }

        [[nodiscard]] angle_type azimuth() const {
            if (has_azimuth()) {
                return geometry::azimuth_segment_deg(_segment);
            }

            throw azimuth_computation_exception{std::format("cannot compute azimuth for segment: {}", wkt())};
        }

        [[nodiscard]] constexpr const segment_type &segment() const {
            return _segment;
        }

        [[nodiscard]] constexpr segment_type &segment() {
            return _segment;
        }

        [[nodiscard]] constexpr const point_type &first() const {
            if constexpr (is_segment<segment_type> or is_referring_segment<segment_type>) {
                return _segment.first;
            } else if constexpr (is_pointing_segment<segment_type>) {
                return *(_segment.first);
            } else {
                static_assert(util::dependent_false_v<segment_type>, "Invalid segment type");
                throw std::runtime_error{"Invalid segment type"};
            }
        }

        [[nodiscard]] constexpr point_type &first() {
            return const_cast<point_type &>(const_cast<const rich_segment *>(this)->first());
        }

        [[nodiscard]] constexpr const point_type &second() const {
            if constexpr (is_segment<segment_type> or is_referring_segment<segment_type>) {
                return _segment.second;
            } else if constexpr (is_pointing_segment<segment_type>) {
                return *(_segment.second);
            } else {
                static_assert(util::dependent_false_v<segment_type>, "Invalid segment type");
                throw std::runtime_error{"Invalid segment type"};
            }
        }

        [[nodiscard]] constexpr point_type &second() {
            return const_cast<point_type &>(const_cast<const rich_segment *>(this)->second());
        }

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(segment());
        }

        [[nodiscard]] std::string str() const {
            std::string str = "RichSegment(";
            str.append(wkt());
            if (has_length()) {
                str.append(",");
                str.append(std::to_string(length()));
            }
            if (has_azimuth()) {
                str.append(",");
                str.append(std::to_string(azimuth()));
            }
            str.append(")");
            return str;
        }

        friend constexpr bool operator==(const rich_segment &left, const rich_segment &right) {
            return geometry::equals_points(left.first(), right.first()) and
                    geometry::equals_points(left.second(), right.second());
        }

        friend constexpr bool operator!=(const rich_segment &left, const rich_segment &right) {
            return not(left == right);
        }

        friend std::size_t hash_value(const rich_segment &data) {
            std::size_t seed = 0;
            boost::hash_combine(seed, data.first());
            boost::hash_combine(seed, data.second());
            return seed;
        }

        friend std::ostream &operator<<(std::ostream &out, const rich_segment rich_segment) {
            return out << rich_segment.str();
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (std::default_initializable<segment_type>) {
            ar & _segment;
        }

    protected:
        segment_type _segment;

    };

}

namespace std {

    template<typename Segment>
    struct hash<map_matching_2::geometry::rich_segment<Segment>> {
        using type = map_matching_2::geometry::rich_segment<Segment>;

        std::size_t operator()(const type &data) const noexcept {
            return hash_value(data);
        }
    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_HPP
