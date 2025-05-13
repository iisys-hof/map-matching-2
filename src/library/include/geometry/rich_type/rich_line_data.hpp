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

#ifndef MAP_MATCHING_2_RICH_TYPE_RICH_LINE_DATA_HPP
#define MAP_MATCHING_2_RICH_TYPE_RICH_LINE_DATA_HPP

#include <format>
#include <vector>

#include <boost/serialization/serialization.hpp>

#include "util/concepts.hpp"

#include "geometry/common.hpp"

#include "geometry/algorithm/equals.hpp"
#include "geometry/algorithm/length.hpp"
#include "geometry/algorithm/azimuth.hpp"
#include "geometry/algorithm/direction.hpp"

#include "common.hpp"
#include "concepts.hpp"

namespace map_matching_2::geometry {

    template<typename Length, typename Angle,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator> requires
        (util::is_random_access_v<Container<Length, Allocator<Length>>> and
            util::is_random_access_v<Container<Angle, Allocator<Angle>>>)
    class rich_line_data {

    public:
        using length_type = Length;
        using angle_type = Angle;

        using allocator_type = Allocator<void>;

        using length_container_type = Container<length_type, Allocator<length_type>>;
        using angle_container_type = Container<angle_type, Allocator<angle_type>>;

        using memory_rich_line_data_type = rich_line_data<length_type, angle_type>;

        constexpr rich_line_data(const rich_line_data &other)
            : _length{other._length}, _azimuth{other._azimuth}, _directions{other._directions},
            _absolute_directions{other._absolute_directions}, _lengths{other._lengths},
            _azimuths{other._azimuths} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line_data(const rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other)
            : _length{other.length()}, _azimuth{other.azimuth()}, _directions{other.directions()},
            _absolute_directions{other.absolute_directions()}, _lengths{},
            _azimuths{} {
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()), std::back_inserter(_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()), std::back_inserter(_azimuths));
        }

        constexpr rich_line_data(const rich_line_data &other, const allocator_type &allocator)
            : _length{other._length}, _azimuth{other._azimuth}, _directions{other._directions},
            _absolute_directions{other._absolute_directions}, _lengths{other._lengths, allocator},
            _azimuths{other._azimuths, allocator} {}

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line_data(const rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other,
                const allocator_type &allocator)
            : _length{other.length()}, _azimuth{other.azimuth()}, _directions{other.directions()},
            _absolute_directions{other.absolute_directions()}, _lengths(allocator),
            _azimuths(allocator) {
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()), std::back_inserter(_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()), std::back_inserter(_azimuths));
        }

        constexpr rich_line_data(rich_line_data &&other) noexcept
            : _length{std::move(other._length)}, _azimuth{std::move(other._azimuth)},
            _directions{std::move(other._directions)},
            _absolute_directions{std::move(other._absolute_directions)}, _lengths{std::move(other._lengths)},
            _azimuths{std::move(other._azimuths)} {}

        constexpr rich_line_data(rich_line_data &&other, const allocator_type &allocator) noexcept
            : _length{std::move(other._length)}, _azimuth{std::move(other._azimuth)},
            _directions{std::move(other._directions)},
            _absolute_directions{std::move(other._absolute_directions)},
            _lengths{std::move(other._lengths), allocator},
            _azimuths{std::move(other._azimuths), allocator} {}

        constexpr rich_line_data &operator=(const rich_line_data &other) {
            if (this != &other) {
                _length = other._length;
                _azimuth = other._azimuth;
                _directions = other._directions;
                _absolute_directions = other._absolute_directions;
                _lengths = other._lengths;
                _azimuths = other._azimuths;
            }
            return *this;
        }

        template<template<typename, typename> typename ContainerT,
            template<typename> typename AllocatorT>
        constexpr rich_line_data &operator=(
                const rich_line_data<length_type, angle_type, ContainerT, AllocatorT> &other) {
            _length = other.length();
            _azimuth = other.azimuth();
            _directions = other.directions();
            _absolute_directions = other.absolute_directions();
            std::copy(std::cbegin(other.lengths()), std::cend(other.lengths()), std::back_inserter(_lengths));
            std::copy(std::cbegin(other.azimuths()), std::cend(other.azimuths()), std::back_inserter(_azimuths));
            return *this;
        }

        constexpr rich_line_data &operator=(rich_line_data &&other) noexcept {
            if (this != &other) {
                _length = std::move(other._length);
                _azimuth = std::move(other._azimuth);
                _directions = std::move(other._directions);
                _absolute_directions = std::move(other._absolute_directions);
                _lengths = std::move(other._lengths);
                _azimuths = std::move(other._azimuths);
            }
            return *this;
        }

        constexpr ~rich_line_data() = default;

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

        [[nodiscard]] constexpr bool has_directions() const {
            return valid(_directions) and valid(_absolute_directions);
        }

        [[nodiscard]] constexpr angle_type directions() const {
            return _directions;
        }

        [[nodiscard]] constexpr angle_type absolute_directions() const {
            return _absolute_directions;
        }

        [[nodiscard]] constexpr const length_container_type &lengths() const {
            return _lengths;
        }

        [[nodiscard]] constexpr length_container_type &lengths() {
            return _lengths;
        }

        [[nodiscard]] constexpr const angle_container_type &azimuths() const {
            return _azimuths;
        }

        [[nodiscard]] constexpr angle_container_type &azimuths() {
            return _azimuths;
        }

        [[nodiscard]] constexpr bool has_length(std::size_t pos) const {
            return valid(_lengths.at(pos));
        }

        [[nodiscard]] constexpr length_type length(std::size_t pos) const {
            return _lengths.at(pos);
        }

        [[nodiscard]] constexpr bool has_azimuth(std::size_t pos) const {
            return valid(_azimuths.at(pos));
        }

        [[nodiscard]] constexpr angle_type azimuth(std::size_t pos) const {
            return _azimuths.at(pos);
        }

        [[nodiscard]] constexpr bool has_direction(std::size_t pos) const {
            if (pos + 1 < _azimuths.size()) {
                return valid(_azimuths[pos]) and valid(_azimuths[pos + 1]);
            }

            throw std::out_of_range(std::format("rich_line_data::has_direction(pos: {}): out of range, size: {}",
                    pos, _azimuths.size()));
        }

        [[nodiscard]] constexpr angle_type direction(std::size_t pos) const {
            if (pos + 1 < _azimuths.size()) {
                return direction_deg(_azimuths[pos], _azimuths[pos + 1]);
            }

            throw std::out_of_range(std::format("rich_line_data::direction(pos: {}): out of range, size: {}",
                    pos, _azimuths.size()));
        }

        [[nodiscard]] constexpr angle_type absolute_direction(std::size_t pos) const {
            return std::fabs(direction(pos));
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) requires
            (std::default_initializable<length_container_type> and std::default_initializable<angle_container_type> and
                std::default_initializable<allocator_type>) {
            ar & _length;
            ar & _azimuth;
            ar & _directions;
            ar & _absolute_directions;
            ar & _lengths;
            ar & _azimuths;
        }

    protected:
        mutable length_type _length;
        mutable angle_type _azimuth;
        mutable angle_type _directions;
        mutable angle_type _absolute_directions;
        mutable length_container_type _lengths;
        mutable angle_container_type _azimuths;

        constexpr rich_line_data() requires
            (std::default_initializable<length_container_type> and std::default_initializable<angle_container_type> and
                std::default_initializable<allocator_type>)
            : _length{not_initialized<length_type>}, _azimuth{not_initialized<angle_type>},
            _directions{not_initialized<length_type>}, _absolute_directions{not_initialized<angle_type>},
            _lengths{}, _azimuths{} {}

        constexpr explicit rich_line_data(const allocator_type &allocator)
            : _length{not_initialized<length_type>}, _azimuth{not_initialized<angle_type>},
            _directions{not_initialized<length_type>}, _absolute_directions{not_initialized<angle_type>},
            _lengths(allocator), _azimuths(allocator) {}

        constexpr rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions) requires
            (std::default_initializable<length_container_type> and std::default_initializable<allocator_type> and
                std::default_initializable<angle_container_type> and std::default_initializable<allocator_type>)
            : _length{std::move(length)}, _azimuth{std::move(azimuth)}, _directions{std::move(directions)},
            _absolute_directions{std::move(absolute_directions)}, _lengths{}, _azimuths{} {}

        constexpr rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, length_container_type lengths,
                angle_container_type azimuths)
            : _length{std::move(length)}, _azimuth{std::move(azimuth)}, _directions{std::move(directions)},
            _absolute_directions{std::move(absolute_directions)}, _lengths{std::move(lengths)},
            _azimuths{std::move(azimuths)} {}

        constexpr rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, const allocator_type &allocator)
            : _length{std::move(length)}, _azimuth{std::move(azimuth)}, _directions{std::move(directions)},
            _absolute_directions{std::move(absolute_directions)}, _lengths(allocator), _azimuths(allocator) {}

        constexpr rich_line_data(length_type length, angle_type azimuth, angle_type directions,
                angle_type absolute_directions, length_container_type lengths,
                angle_container_type azimuths, const allocator_type &allocator)
            : _length{std::move(length)}, _azimuth{std::move(azimuth)}, _directions{std::move(directions)},
            _absolute_directions{std::move(absolute_directions)}, _lengths{std::move(lengths), allocator},
            _azimuths{std::move(azimuths), allocator} {}

        void _invalidate(bool length = true, bool azimuth = true, bool directions = true,
                bool lengths = true, bool azimuths = true) {
            if (length) {
                _length = not_initialized<length_type>;
            }
            if (azimuth) {
                _azimuth = not_initialized<angle_type>;
            }
            if (directions) {
                _directions = not_initialized<length_type>;
                _absolute_directions = not_initialized<angle_type>;
            }
            if (lengths) {
                _lengths.clear();
            }
            if (azimuths) {
                _azimuths.clear();
            }
        }

        void _compute_length(const is_rich_line auto &rich_line) const {
            if (not initialized(_length)) {
                _length = not_valid<length_type>;

                _compute_lengths(rich_line);

                bool has_length = false;
                length_type length = default_float_type<length_type>::v0;

                for (std::size_t i = 0; i < _lengths.size(); ++i) {
                    length_type &segment_length = _lengths[i];
                    if (not initialized(segment_length)) {
                        segment_length = _compute_segment_length(rich_line, i);
                    }
                    if (valid(segment_length)) {
                        length += segment_length;
                        has_length = true;
                    }
                }

                if (has_length) {
                    _length = length;
                }
            }
        }

        [[nodiscard]] length_type _compute_segment_length(
                const is_rich_line auto &rich_line, std::size_t index) const {
            const auto segment = rich_line.referring_segment(index);
            length_type length = not_valid<length_type>;
            if (not geometry::equals_points_segment(segment)) {
                length = geometry::length_segment(segment);
            }
            return length;
        }

        void _compute_lengths(const is_rich_line auto &rich_line) const {
            if (_lengths.empty()) {
                if (rich_line.size() >= 2) {
                    _lengths.reserve(rich_line.size() - 1);
                    for (std::size_t i = 0; i < rich_line.size() - 1; ++i) {
                        _lengths.emplace_back(_compute_segment_length(rich_line, i));
                    }

                    _lengths.shrink_to_fit();
                }
            }
        }

        void _initialize_length(const is_rich_line auto &rich_line, std::size_t pos) const {
            if (not initialized(_lengths.at(pos))) {
                _lengths[pos] = _compute_segment_length(rich_line, pos);
            }
        }

        void _compute_azimuth(const is_rich_line auto &rich_line) const {
            if (not initialized(_azimuth)) {
                _azimuth = not_valid<angle_type>;

                if (rich_line.size() >= 2) {
                    const auto &start = rich_line.front();
                    const auto &end = rich_line.back();
                    if (not geometry::equals_points(start, end)) {
                        _azimuth = geometry::azimuth_deg(start, end);
                    }
                }
            }
        }

        [[nodiscard]] angle_type _compute_segment_azimuth(
                const is_rich_line auto &rich_line, std::size_t index) const {
            const auto segment = rich_line.referring_segment(index);
            angle_type azimuth = not_valid<angle_type>;
            if (not geometry::equals_points_segment(segment)) {
                azimuth = geometry::azimuth_segment_deg(segment);
            }
            return azimuth;
        }

        void _compute_azimuths(const is_rich_line auto &rich_line) const {
            if (_azimuths.empty()) {
                if (rich_line.size() >= 2) {
                    _azimuths.reserve(rich_line.size() - 1);
                    for (std::size_t i = 0; i < rich_line.size() - 1; ++i) {
                        _azimuths.emplace_back(_compute_segment_azimuth(rich_line, i));
                    }

                    _azimuths.shrink_to_fit();
                }
            }
        }

        void _initialize_azimuth(const is_rich_line auto &rich_line, std::size_t pos) const {
            if (not initialized(_azimuths.at(pos))) {
                _azimuths[pos] = _compute_segment_azimuth(rich_line, pos);
            }
        }

        void _compute_directions(const is_rich_line auto &rich_line) const {
            if (not initialized(_directions) or not initialized(_absolute_directions)) {
                _directions = not_valid<angle_type>;
                _absolute_directions = not_valid<angle_type>;

                _compute_azimuths(rich_line);

                if (_azimuths.size() >= 2) {
                    bool has_directions = false;
                    angle_type directions = default_float_type<angle_type>::v0;
                    angle_type absolute_directions = default_float_type<angle_type>::v0;

                    for (std::size_t i = 0; i < _azimuths.size() - 1; ++i) {
                        angle_type &azimuth_a = _azimuths[i];
                        angle_type &azimuth_b = _azimuths[i + 1];
                        if (not initialized(azimuth_a)) {
                            azimuth_a = _compute_segment_azimuth(rich_line, i);
                        }
                        if (not initialized(azimuth_b)) {
                            azimuth_b = _compute_segment_azimuth(rich_line, i + 1);
                        }
                        if (valid(azimuth_a) and valid(azimuth_b)) {
                            angle_type direction = geometry::direction_deg(azimuth_a, azimuth_b);
                            directions += direction;
                            absolute_directions += std::fabs(direction);
                            has_directions = true;
                        }
                    }

                    if (has_directions) {
                        _directions = directions;
                        _absolute_directions = absolute_directions;
                    }
                }
            }
        }

        void _initialize_direction(const is_rich_line auto &rich_line, std::size_t pos) const {
            if (pos + 1 < _azimuths.size()) {
                _initialize_azimuth(rich_line, pos);
                _initialize_azimuth(rich_line, pos + 1);
            }

            throw std::out_of_range(std::format(
                    "rich_line_data::_initialize_direction(pos: {}): out of range, size: {}",
                    pos, _azimuths.size()));
        }

        void _compute(const is_rich_line auto &rich_line) const {
            _compute_length(rich_line);
            _compute_azimuth(rich_line);
            _compute_directions(rich_line);
        }

        void _rebuild(const is_rich_line auto &rich_line,
                bool length = true, bool azimuth = true, bool directions = true) const {
            if (length) {
                _length = not_initialized<length_type>;
                _compute_length(rich_line);
            }
            if (azimuth) {
                _azimuth = not_initialized<angle_type>;
                _compute_azimuth(rich_line);
            }
            if (directions) {
                _directions = not_initialized<length_type>;
                _absolute_directions = not_initialized<angle_type>;
                _compute_directions(rich_line);
            }
        }

        [[nodiscard]] rich_line_data sub_data(std::size_t from, std::size_t to) const requires
            (std::default_initializable<allocator_type> and std::default_initializable<allocator_type>) {
            if (to > 0) {
                to--;
            }
            if (from >= to) {
                return rich_line_data{};
            }
            return rich_line_data{
                    (from == 0 and to + 1 >= _lengths.size()) ? _length : not_initialized<length_type>,
                    (from == 0 and to + 1 >= _lengths.size()) ? _azimuth : not_initialized<angle_type>,
                    (from == 0 and to + 1 >= _azimuths.size()) ? _directions : not_initialized<angle_type>,
                    (from == 0 and to + 1 >= _azimuths.size()) ? _absolute_directions : not_initialized<angle_type>,
                    (not _lengths.empty() and from <= to and from < _lengths.size() and to <= _lengths.size())
                    ? length_container_type{_lengths.cbegin() + from, _lengths.cbegin() + to}
                    : length_container_type{},
                    (not _azimuths.empty() and from <= to and from < _azimuths.size() and to <= _azimuths.size())
                    ? angle_container_type{_azimuths.cbegin() + from, _azimuths.cbegin() + to}
                    : angle_container_type{}
            };
        }

        [[nodiscard]] rich_line_data sub_data(std::size_t from, std::size_t to,
                const allocator_type &allocator) const {
            if (to > 0) {
                to--;
            }
            if (from >= to) {
                return rich_line_data{allocator};
            }
            return rich_line_data{
                    (from == 0 and to + 1 >= _lengths.size()) ? _length : not_initialized<length_type>,
                    (from == 0 and to + 1 >= _lengths.size()) ? _azimuth : not_initialized<angle_type>,
                    (from == 0 and to + 1 >= _azimuths.size()) ? _directions : not_initialized<angle_type>,
                    (from == 0 and to + 1 >= _azimuths.size()) ? _absolute_directions : not_initialized<angle_type>,
                    (not _lengths.empty() and from <= to and from < _lengths.size() and to <= _lengths.size())
                    ? length_container_type{_lengths.cbegin() + from, _lengths.cbegin() + to, allocator}
                    : length_container_type{allocator},
                    (not _azimuths.empty() and from <= to and from < _azimuths.size() and to <= _azimuths.size())
                    ? angle_container_type{_azimuths.cbegin() + from, _azimuths.cbegin() + to, allocator}
                    : angle_container_type{allocator},
                    allocator
            };
        }

        template<is_rich_line RichLine>
        void _merge_single(RichLine &&next_rich_line) {
            auto &&_data = static_cast<rich_line_data &&>(next_rich_line);

            if (not next_rich_line.empty()) {
                _data._lengths.reserve(next_rich_line.size() - 1);
                while (next_rich_line.size() - 1 > _data._lengths.size()) {
                    _data._lengths.emplace_back(not_initialized<length_type>);
                }
                _data._azimuths.reserve(next_rich_line.size() - 1);
                while (next_rich_line.size() - 1 > _data._azimuths.size()) {
                    _data._azimuths.emplace_back(not_initialized<angle_type>);
                }
            }

            if (not _data._lengths.empty()) {
                _lengths.reserve(_lengths.size() + _data._lengths.size());
                std::move(std::begin(_data._lengths), std::end(_data._lengths), std::back_inserter(_lengths));
            }
            if (not _data._azimuths.empty()) {
                _azimuths.reserve(_azimuths.size() + _data._azimuths.size());
                std::move(std::begin(_data._azimuths), std::end(_data._azimuths), std::back_inserter(_azimuths));
            }
        }

        template<is_rich_line RichLine>
        void _reserve(RichLine &rich_line) {
            if (not rich_line.empty()) {
                _lengths.reserve(rich_line.size() - 1);
                while (rich_line.size() - 1 > _lengths.size()) {
                    _lengths.emplace_back(not_initialized<length_type>);
                }
                _azimuths.reserve(rich_line.size() - 1);
                while (rich_line.size() - 1 > _azimuths.size()) {
                    _azimuths.emplace_back(not_initialized<angle_type>);
                }
            }
        }

        template<is_rich_line RichLine, is_rich_line... RichLines>
        rich_line_data &_merge(RichLine &rich_line, RichLines &&... rich_lines) requires
            (std::is_base_of_v<rich_line_data, RichLine> && (... && std::is_base_of_v<rich_line_data, RichLines>)) {
            _reserve(rich_line);
            (_merge_single(std::move(rich_lines)), ...);
            return *this;
        }

        template<is_rich_line RichLine>
        rich_line_data &_merge(RichLine &rich_line, std::vector<RichLine> &&rich_lines) requires
            (std::is_base_of_v<rich_line_data, RichLine>) {
            using rich_line_type = RichLine;

            _reserve(rich_line);
            for (rich_line_type &next_rich_line : rich_lines) {
                _merge_single(std::move(next_rich_line));
            }

            return *this;
        }

        void _erase_and_replace_length(const is_rich_line auto &rich_line, std::size_t index) const {
            if (not _lengths.empty() and index <= _lengths.size()) {
                _lengths.erase(std::next(_lengths.begin(), index < _lengths.size() ? index : index - 1));
                if (not _lengths.empty() and index > 0 and index < rich_line.size()) {
                    _lengths[index - 1] = _compute_segment_length(rich_line, index - 1);
                }
            }
        }

        void _erase_and_replace_azimuth(const is_rich_line auto &rich_line, std::size_t index) const {
            if (not _azimuths.empty() and index <= _azimuths.size()) {
                _azimuths.erase(std::next(_azimuths.begin(), index < _azimuths.size() ? index : index - 1));
                if (not _azimuths.empty() and index > 0 and index < rich_line.size()) {
                    _azimuths[index - 1] = _compute_segment_azimuth(rich_line, index - 1);
                }
            }
        }

        void _erase_and_replace(const is_rich_line auto &rich_line, std::size_t index) const {
            _erase_and_replace_length(rich_line, index);
            _erase_and_replace_azimuth(rich_line, index);
        }

    };

}

#endif //MAP_MATCHING_2_RICH_TYPE_RICH_LINE_DATA_HPP
