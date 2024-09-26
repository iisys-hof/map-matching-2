// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_DATA_HPP
#define MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_DATA_HPP

#include <boost/serialization/serialization.hpp>

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/length.hpp"
#include "geometry/algorithm/azimuth.hpp"

#include "common.hpp"
#include "concepts.hpp"

namespace map_matching_2::geometry {

    template<typename Length, typename Angle>
    class rich_segment_data {

    public:
        using length_type = Length;
        using angle_type = Angle;

        constexpr rich_segment_data(const rich_segment_data &other)
            : _length{other._length}, _azimuth{other._azimuth} {}

        constexpr rich_segment_data(rich_segment_data &&other) noexcept
            : _length{std::move(other._length)}, _azimuth{std::move(other._azimuth)} {}

        constexpr rich_segment_data &operator=(const rich_segment_data &other) {
            if (this != &other) {
                _length = other._length;
                _azimuth = other._azimuth;
            }
            return *this;
        }

        constexpr rich_segment_data &operator=(rich_segment_data &&other) noexcept {
            if (this != &other) {
                _length = std::move(other._length);
                _azimuth = std::move(other._azimuth);
            }
            return *this;
        }

        constexpr ~rich_segment_data() = default;

        [[nodiscard]] constexpr bool has_length() const {
            return valid(_length);
        }

        [[nodiscard]] constexpr length_type length() const {
            return _length;
        }

        [[nodiscard]] constexpr bool has_azimuth() const {
            return valid(_azimuth);
        }

        [[nodiscard]] constexpr angle_type azimuth() const {
            return _azimuth;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _length;
            ar & _azimuth;
        }

    protected:
        mutable length_type _length{not_initialized<length_type>};
        mutable angle_type _azimuth{not_initialized<length_type>};

        constexpr rich_segment_data() = default;

        constexpr rich_segment_data(length_type length, angle_type azimuth)
            : _length{length}, _azimuth{azimuth} {}

        [[nodiscard]] constexpr bool _equals_points(const is_rich_segment auto &rich_segment) const {
            return geometry::equals_points(rich_segment.first(), rich_segment.second());
        }

        void _compute_length(const is_rich_segment auto &rich_segment) const {
            if (not initialized(_length)) {
                _length = not_valid<length_type>;
                if (not _equals_points(rich_segment)) {
                    _length = geometry::length_segment(rich_segment.segment());
                }
            }
        }

        void _compute_azimuth(const is_rich_segment auto &rich_segment) const {
            if (not initialized(_azimuth)) {
                _azimuth = not_valid<angle_type>;
                if (not _equals_points(rich_segment)) {
                    _azimuth = geometry::azimuth_segment_deg(rich_segment.segment());
                }
            }
        }

        void _compute(const is_rich_segment auto &rich_segment) const {
            _compute_length(rich_segment);
            _compute_azimuth(rich_segment);
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_RICH_SEGMENT_DATA_HPP
