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

#ifndef MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP
#define MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP

#include <vector>

#include <boost/serialization/serialization.hpp>

#include "common.hpp"
#include "concepts.hpp"

#include "rich_line_data.hpp"

namespace map_matching_2::geometry {

    template<typename Length, typename Angle,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator> requires
        (util::is_random_access_v<Container<Length, Allocator<Length>>> and
            util::is_random_access_v<Container<Angle, Allocator<Angle>>>)
    class multi_rich_line_data : public rich_line_data<Length, Angle, Container, Allocator> {

    public:
        using length_type = Length;
        using angle_type = Angle;

        using base_type = rich_line_data<Length, Angle, Container, Allocator>;

        using allocator_type = Allocator<void>;

        using length_container_type = Container<length_type, Allocator<length_type>>;
        using angle_container_type = Container<angle_type, Allocator<angle_type>>;

        constexpr multi_rich_line_data(const multi_rich_line_data &other)
            : base_type{other} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr multi_rich_line_data(
                const multi_rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other)
            : base_type{other} {}

        constexpr multi_rich_line_data(const multi_rich_line_data &other, const allocator_type &allocator)
            : base_type{other, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr multi_rich_line_data(
                const multi_rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : base_type{other, allocator} {}

        constexpr multi_rich_line_data(multi_rich_line_data &&other) noexcept
            : base_type{std::move(other)} {}

        constexpr multi_rich_line_data(multi_rich_line_data &&other, const allocator_type &allocator) noexcept
            : base_type{std::move(other), allocator} {}

        constexpr multi_rich_line_data &operator=(const multi_rich_line_data &other) {
            if (this != &other) {
                base_type::operator=(other);
            }
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr multi_rich_line_data &operator=(
                const multi_rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other) {
            base_type::operator=(other);
            return *this;
        }

        constexpr multi_rich_line_data &operator=(multi_rich_line_data &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr ~multi_rich_line_data() = default;

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<base_type>(*this);
        }

    protected:
        constexpr multi_rich_line_data() requires
            (std::default_initializable<length_container_type> and std::default_initializable<angle_container_type> and
                std::default_initializable<allocator_type>)
            : base_type{} {}

        constexpr multi_rich_line_data(const allocator_type &allocator)
            : base_type(allocator) {}

        constexpr multi_rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, length_container_type lengths, angle_container_type azimuths)
            : base_type{
                    std::move(length), std::move(azimuth), std::move(directions),
                    std::move(absolute_directions), std::move(lengths), std::move(azimuths)
            } {}

        constexpr multi_rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, length_container_type lengths, angle_container_type azimuths,
                const allocator_type &allocator)
            : base_type{
                    std::move(length), std::move(azimuth), std::move(directions),
                    std::move(absolute_directions), std::move(lengths), std::move(azimuths), allocator
            } {}

        void _compute_length(const is_multi_rich_line auto &multi_rich_line) const {
            if (not initialized(this->_length)) {
                this->_length = not_valid<length_type>;

                _compute_lengths(multi_rich_line);

                bool has_length = false;
                length_type length = default_float_type<length_type>::v0;

                for (length_type rich_line_length : this->_lengths) {
                    if (valid(rich_line_length)) {
                        length += rich_line_length;
                        has_length = true;
                    }
                }

                if (has_length) {
                    this->_length = length;
                }
            }
        }

        void _compute_lengths(const is_multi_rich_line auto &multi_rich_line) const {
            if (this->_lengths.empty()) {
                if (not multi_rich_line.empty()) {
                    this->_lengths.reserve(multi_rich_line.size());
                    for (const auto &rich_line : multi_rich_line.rich_lines()) {
                        length_type length = not_valid<length_type>;
                        if (rich_line.has_length()) {
                            length = rich_line.length();
                        }
                        this->_lengths.emplace_back(length);
                    }

                    this->_lengths.shrink_to_fit();
                }
            }
        }

        void _compute_azimuth(const is_multi_rich_line auto &multi_rich_line) const {
            if (not initialized(this->_azimuth)) {
                this->_azimuth = not_valid<angle_type>;

                if (not multi_rich_line.empty()) {
                    if (multi_rich_line.size() == 1) {
                        const auto &rich_line = multi_rich_line.front();
                        if (rich_line.has_azimuth()) {
                            this->_azimuth = rich_line.azimuth();
                        }
                    } else {
                        const auto &start = multi_rich_line.front().front();
                        const auto &end = multi_rich_line.back().back();
                        if (not geometry::equals_points(start, end)) {
                            this->_azimuth = geometry::azimuth_deg(start, end);
                        }
                    }
                }
            }
        }

        void _compute_azimuths(const is_multi_rich_line auto &multi_rich_line) const {
            if (this->_azimuths.empty()) {
                if (multi_rich_line.size() >= 2) {
                    this->_azimuths.reserve(multi_rich_line.size() - 1);

                    for (std::size_t i = 0; i < multi_rich_line.size() - 1; ++i) {
                        const auto &start = multi_rich_line[i];
                        const auto &end = multi_rich_line[i + 1];

                        if (geometry::equals_points(start.back(), end.front()) and
                            start.size() >= 2 and start.has_azimuth(start.size() - 2) and
                            end.size() >= 2 and end.has_azimuth(0)) {
                            // used for directions between lines here
                            this->_azimuths.emplace_back(geometry::direction_deg(
                                    start.azimuth(start.size() - 2), end.azimuth(0)));
                        } else {
                            this->_azimuths.emplace_back(not_valid<angle_type>);
                        }
                    }

                    this->_azimuths.shrink_to_fit();
                }
            }
        }

        void _compute_directions(const is_multi_rich_line auto &multi_rich_line) const {
            if (not initialized(this->_directions) or not initialized(this->_absolute_directions)) {
                this->_directions = not_valid<angle_type>;
                this->_absolute_directions = not_valid<angle_type>;

                _compute_azimuths(multi_rich_line);

                if (not multi_rich_line.empty()) {
                    assert(multi_rich_line.size() - 1 == this->_azimuths.size());

                    bool has_directions = false;
                    angle_type directions = default_float_type<angle_type>::v0;
                    angle_type absolute_directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0; i < multi_rich_line.size(); ++i) {
                        const auto &rich_line = multi_rich_line[i];
                        if (rich_line.has_directions()) {
                            directions += rich_line.directions();
                            absolute_directions += rich_line.absolute_directions();
                            has_directions = true;
                        }
                        if (i < this->_azimuths.size() and valid(this->_azimuths[i])) {
                            // contains here a direction change, not an azimuth
                            angle_type direction = this->_azimuths[i];
                            directions += direction;
                            absolute_directions += std::fabs(direction);
                            has_directions = true;
                        }
                    }

                    if (has_directions) {
                        this->_directions = directions;
                        this->_absolute_directions = absolute_directions;
                    }
                }
            }
        }

        void _compute(const is_multi_rich_line auto &multi_rich_line) const {
            _compute_length(multi_rich_line);
            _compute_azimuth(multi_rich_line);
            _compute_directions(multi_rich_line);
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_MULTI_RICH_LINE_DATA_HPP
