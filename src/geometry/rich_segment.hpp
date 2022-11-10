// Copyright (C) 2021-2021 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_RICH_SEGMENT_HPP
#define MAP_MATCHING_2_RICH_SEGMENT_HPP

#include <atomic>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "traits.hpp"
#include "util.hpp"

namespace map_matching_2::geometry {

    template<typename RichTypeA, typename RichTypeB, template<typename> class Container>
    [[nodiscard]] static Container<RichTypeA> convert_rich_types(const Container<RichTypeB> &rich_segments) {
        Container<RichTypeA> container;
        if constexpr (std::is_void_v<decltype(&Container<RichTypeA>::reserve)>) {
            container.reserve(rich_segments.size());
        }
        for (const RichTypeB &rich_segment: rich_segments) {
            container.emplace_back(RichTypeA{rich_segment});
        }
        return container;
    }

    template<typename RichTypeA, typename RichTypeB, template<typename> class Container>
    [[nodiscard]] static Container<RichTypeA> convert_rich_types(Container<RichTypeB> &&rich_segments) {
        Container<RichTypeA> container;
        if constexpr (std::is_void_v<decltype(&Container<RichTypeA>::reserve)>) {
            container.reserve(rich_segments.size());
        }
        for (RichTypeB &rich_segment: rich_segments) {
            container.emplace_back(RichTypeA{std::move(rich_segment)});
        }
        return container;
    }

    template<typename RichSegment,
            template<typename> class ContainerA,
            template<typename> class ContainerB>
    [[nodiscard]] static ContainerA<RichSegment>
    points2rich_segments(const ContainerB<typename rich_segment_traits<RichSegment>::point_type> &points) {
        using segment_type = typename rich_segment_traits<RichSegment>::segment_type;

        ContainerA<RichSegment> rich_segments;
        if (points.size() >= 2) {
            if constexpr (std::is_void_v<decltype(&ContainerA<RichSegment>::reserve)>) {
                rich_segments.reserve(points.size() - 1);
            }
            for (auto it = points.cbegin(); std::next(it) != points.cend(); it++) {
                rich_segments.emplace_back(RichSegment{segment_type{*it, *std::next(it)}});
            }
        }
        return rich_segments;
    }

    template<typename RichSegment,
            template<typename> class ContainerA,
            template<typename> class ContainerB>
    [[nodiscard]] static ContainerA<typename rich_segment_traits<RichSegment>::point_type>
    rich_segments2points(const ContainerB<RichSegment> &rich_segments) {
        using point_type = typename rich_segment_traits<RichSegment>::point_type;
        using segment_type = typename rich_segment_traits<RichSegment>::segment_type;

        ContainerA<point_type> points;
        if (not rich_segments.empty()) {
            if constexpr (std::is_void_v<decltype(&ContainerA<point_type>::reserve)>) {
                points.reserve(rich_segments.size() + 1);
            }
            for (const auto &rich_segment: rich_segments) {
                if (points.empty()) {
                    points.emplace_back(rich_segment.first());
                }
                points.emplace_back(rich_segment.second());
            }
        }
        return points;
    }

    template<typename RichSegment, template<typename> class Container>
    [[nodiscard]] static Container<RichSegment>
    line2rich_segments(const typename rich_segment_traits<RichSegment>::line_type &line) {
        using segment_type = typename rich_segment_traits<RichSegment>::segment_type;

        Container<RichSegment> rich_segments;
        if (line.size() >= 2) {
            if constexpr (std::is_void_v<decltype(&Container<RichSegment>::reserve)>) {
                rich_segments.reserve(line.size() - 1);
            }
            for (std::size_t i = 0; i < line.size() - 1; ++i) {
                rich_segments.emplace_back(RichSegment{segment_type{line[i], line[i + 1]}});
            }
        }
        return rich_segments;
    }

    template<typename RichSegment, template<typename> class Container>
    [[nodiscard]] static typename rich_segment_traits<RichSegment>::line_type
    rich_segments2line(const Container<RichSegment> &rich_segments) {
        using line_type = typename rich_segment_traits<RichSegment>::line_type;

        line_type line;
        if (not rich_segments.empty()) {
            line.reserve(rich_segments.size() + 1);
            for (const auto &rich_segment: rich_segments) {
                if (line.empty()) {
                    line.emplace_back(rich_segment.first());
                }
                line.emplace_back(rich_segment.second());
            }
        }
        return line;
    }

    template<typename Segment>
    class lazy_base_rich_segment;

    template<typename Segment>
    class base_rich_segment {

    public:
        using segment_type = Segment;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;

        ~base_rich_segment() = default;

        base_rich_segment(const base_rich_segment &other) {
            _transfer(other);
        }

        template<typename SegmentT>
        base_rich_segment(const base_rich_segment<SegmentT> &other) {
            _transfer(other);
        }

        template<typename SegmentT>
        base_rich_segment(const lazy_base_rich_segment<SegmentT> &other) {
            _transfer(other);
        }

        base_rich_segment(base_rich_segment &&other) {
            _transfer(other);
        }

        template<typename SegmentT>
        base_rich_segment(base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
        }

        template<typename SegmentT>
        base_rich_segment(lazy_base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
        }

        base_rich_segment &operator=(const base_rich_segment &other) {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        base_rich_segment &operator=(const base_rich_segment<SegmentT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        base_rich_segment &operator=(const lazy_base_rich_segment<SegmentT> &other) {
            _transfer(other);
            return *this;
        }

        base_rich_segment &operator=(base_rich_segment &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        base_rich_segment &operator=(base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        base_rich_segment &operator=(lazy_base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            return _has_length;
        }

        [[nodiscard]] length_type length() const {
            return _length;
        }

        [[nodiscard]] bool has_azimuth() const {
            return _has_azimuth;
        }

        [[nodiscard]] angle_type azimuth() const {
            return _azimuth;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _has_length;
            ar & _has_azimuth;
            ar & _length;
            ar & _azimuth;
        }

    protected:
        mutable length_type _length;
        mutable angle_type _azimuth;
        mutable bool _has_length, _has_azimuth;

        base_rich_segment()
                : _has_length{false}, _has_azimuth{false} {}

        base_rich_segment(length_type length, angle_type azimuth)
                : _has_length{true}, _length{length}, _has_azimuth{true}, _azimuth{azimuth} {}

        template<typename SegmentT>
        void _transfer(const base_rich_segment<SegmentT> &other) {
            if (other.has_length()) {
                this->_has_length = other.has_length();
                this->_length = other.length();
            }
            if (other.has_azimuth()) {
                this->_has_azimuth = other.has_azimuth();
                this->_azimuth = other.azimuth();
            }
        }

    };

    template<typename Segment>
    class lazy_base_rich_segment : public base_rich_segment<Segment> {

    public:
        using segment_type = Segment;

        ~lazy_base_rich_segment() = default;

        lazy_base_rich_segment(const lazy_base_rich_segment &other)
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        template<typename SegmentT>
        lazy_base_rich_segment(const lazy_base_rich_segment<SegmentT> &other)
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        template<typename SegmentT>
        lazy_base_rich_segment(const base_rich_segment<SegmentT> &other)
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        lazy_base_rich_segment(lazy_base_rich_segment &&other) noexcept
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        template<typename SegmentT>
        lazy_base_rich_segment(lazy_base_rich_segment<SegmentT> &&other) noexcept
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        template<typename SegmentT>
        lazy_base_rich_segment(base_rich_segment<SegmentT> &&other) noexcept
                : base_rich_segment<segment_type>{} {
            _transfer(other);
        }

        lazy_base_rich_segment &operator=(const lazy_base_rich_segment &other) {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_base_rich_segment &operator=(const lazy_base_rich_segment<SegmentT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_base_rich_segment &operator=(const base_rich_segment<SegmentT> &other) {
            _transfer(other);
            return *this;
        }

        lazy_base_rich_segment &operator=(lazy_base_rich_segment &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_base_rich_segment &operator=(lazy_base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_base_rich_segment &operator=(base_rich_segment<SegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        [[nodiscard]] std::atomic<bool> &computed_length() const {
            return _computed_length;
        }

        [[nodiscard]] std::atomic<bool> &computed_azimuth() const {
            return _computed_azimuth;
        }

        [[nodiscard]] std::mutex &mutex() const {
            return _mutex;
        }

    protected:
        mutable std::atomic<bool> _computed_length, _computed_azimuth;
        mutable std::mutex _mutex;

        lazy_base_rich_segment()
                : base_rich_segment<segment_type>{}, _computed_length{false}, _computed_azimuth{false} {}

        template<typename SegmentT>
        void _transfer(const lazy_base_rich_segment<SegmentT> &other) {
            _computed_length.store(false, std::memory_order_release);
            if (other.computed_length().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(_mutex);
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_length().load(std::memory_order_relaxed)) {
                    this->_length = other.length();
                    this->_has_length = other.has_length();
                    _computed_length.store(true, std::memory_order_release);
                }
            }

            _computed_azimuth.store(false, std::memory_order_release);
            if (other.computed_azimuth().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(_mutex);
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_azimuth().load(std::memory_order_relaxed)) {
                    this->_azimuth = other.azimuth();
                    this->_has_azimuth = other.has_azimuth();
                    _computed_azimuth.store(true, std::memory_order_release);
                }
            }
        }

        template<typename SegmentT>
        void _transfer(const base_rich_segment<SegmentT> &other) {
            _computed_length.store(false, std::memory_order_release);
            if (other.has_length()) {
                std::lock_guard<std::mutex> lock(_mutex);
                if (other.has_length()) {
                    this->_length = other.length();
                    this->_has_length = other.has_length();
                    _computed_length.store(true, std::memory_order_release);
                }
            }

            _computed_azimuth.store(false, std::memory_order_release);
            if (other.has_azimuth()) {
                std::lock_guard<std::mutex> lock(_mutex);
                if (other.has_azimuth()) {
                    this->_azimuth = other.azimuth();
                    this->_has_azimuth = other.has_azimuth();
                    _computed_azimuth.store(true, std::memory_order_release);
                }
            }
        }

    };

    template<typename Segment>
    class eager_rich_segment;

    template<typename Segment>
    class lazy_rich_segment;

    template<typename Segment>
    class rich_segment {

    public:
        using segment_type = Segment;
        using point_type = point_type_t<segment_type>;
        using line_type = typename models<point_type>::line_type;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;

        template<typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        rich_segment()
                : _segment{} {}

        explicit rich_segment(segment_type segment)
                : _segment{std::move(segment)} {}

        ~rich_segment() = default;

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const rich_segment &other)
                : _segment{other.segment()} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const rich_segment<SegmentT> &other)
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const eager_rich_segment<SegmentT> &other)
                : _segment{other.segment()} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const eager_rich_segment<SegmentT> &other)
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const lazy_rich_segment<SegmentT> &other)
                : _segment{other.segment()} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(const lazy_rich_segment<SegmentT> &other)
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(rich_segment &&other) noexcept
                : _segment{std::move(const_cast<segment_type &>(other.segment()))} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(rich_segment<SegmentT> &&other) noexcept
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(eager_rich_segment<SegmentT> &&other) noexcept
                : _segment{std::move(const_cast<segment_type &>(other.segment()))} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(eager_rich_segment<SegmentT> &&other) noexcept
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_constructible_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(lazy_rich_segment<SegmentT> &&other) noexcept
                : _segment{std::move(const_cast<segment_type &>(other.segment()))} {}

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_constructible_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment(lazy_rich_segment<SegmentT> &&other) noexcept
                : _segment{segment_type{other.first(), other.second()}} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const rich_segment &other) {
            _segment = other.segment();
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const rich_segment <SegmentT> &other) {
            const_cast<point_type &>(first()) = other.first();
            const_cast<point_type &>(second()) = other.second();
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const eager_rich_segment<SegmentT> &other) {
            _segment = other.segment();
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const eager_rich_segment<SegmentT> &other) {
            const_cast<point_type &>(first()) = other.first();
            const_cast<point_type &>(second()) = other.second();
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const lazy_rich_segment<SegmentT> &other) {
            _segment = other.segment();
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_copy_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(const lazy_rich_segment<SegmentT> &other) {
            const_cast<point_type &>(first()) = other.first();
            const_cast<point_type &>(second()) = other.second();
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(rich_segment &&other) noexcept {
            _segment = std::move(const_cast<segment_type &>(other.segment()));
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(rich_segment <SegmentT> &&other) noexcept {
            const_cast<point_type &>(first()) = std::move(const_cast<point_type &>(other.first()));
            const_cast<point_type &>(second()) = std::move(const_cast<point_type &>(other.second()));
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(eager_rich_segment<SegmentT> &&other) noexcept {
            _segment = std::move(const_cast<segment_type &>(other.segment()));
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(eager_rich_segment<SegmentT> &&other) noexcept {
            const_cast<point_type &>(first()) = std::move(const_cast<point_type &>(other.first()));
            const_cast<point_type &>(second()) = std::move(const_cast<point_type &>(other.second()));
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_assignable_v<SegmentT> &&
                                 std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(lazy_rich_segment<SegmentT> &&other) noexcept {
            _segment = std::move(const_cast<segment_type &>(other.segment()));
            return *this;
        }

        template<typename SegmentT,
                std::enable_if_t<!std::is_move_assignable_v<SegmentT> ||
                                 !std::is_same_v<segment_type, SegmentT>, bool> = true>
        rich_segment &operator=(lazy_rich_segment<SegmentT> &&other) noexcept {
            const_cast<point_type &>(first()) = std::move(const_cast<point_type &>(other.first()));
            const_cast<point_type &>(second()) = std::move(const_cast<point_type &>(other.second()));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            return not geometry::equals_points(this->first(), this->second());
        }

        [[nodiscard]] length_type length() const {
            if (has_length()) {
                return geometry::length_segment(this->segment());
            }
            throw std::runtime_error("segment has no length");
        }

        [[nodiscard]] bool has_azimuth() const {
            return not geometry::equals_points(this->first(), this->second());
        }

        [[nodiscard]] angle_type azimuth() const {
            if (has_azimuth()) {
                return geometry::azimuth_segment_deg(this->segment());
            }
            throw std::runtime_error("segment has no azimuth");
        }

        [[nodiscard]] const segment_type &segment() const {
            return _segment;
        }

        [[nodiscard]] const point_type &first() const {
            if constexpr (is_segment_v<segment_type> or is_referring_segment_v<segment_type>) {
                return _segment.first;
            } else if constexpr (is_pointing_segment_v<segment_type>) {
                return *(_segment.first);
            } else {
                static_assert(dependent_false_v<segment_type>, "Invalid segment type");
                throw std::runtime_error{"Invalid segment type"};
            }
        }

        [[nodiscard]] const point_type &second() const {
            if constexpr (is_segment_v<segment_type> or is_referring_segment_v<segment_type>) {
                return _segment.second;
            } else if constexpr (is_pointing_segment_v<segment_type>) {
                return *(_segment.second);
            } else {
                static_assert(dependent_false_v<segment_type>, "Invalid segment type");
                throw std::runtime_error{"Invalid segment type"};
            }
        }

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(segment());
        }

        [[nodiscard]] std::string str() const {
            std::string str = "RichSegment(";
            str.append(wkt());
            str.append(")");
            return str;
        }

        template<typename RichSegment>
        static void simplify(std::list<RichSegment> &segments, bool retain_reversals = true, double tolerance = 1e-3,
                             double reverse_tolerance = 1e-3) {
            if (not retain_reversals) {
                _simplify(segments, segments.begin(), segments.end(), tolerance);
            } else {
                auto index = segments.begin();
                for (auto it = segments.begin(); it != segments.end(); it++) {
                    if (std::next(it) != segments.end()) {
                        auto next_it = std::next(it);
                        auto &segment = *it;
                        auto &next_segment = *next_it;
                        if (segment.has_azimuth() and next_segment.has_azimuth()) {
                            const auto direction =
                                    std::fabs(geometry::direction_deg(segment.azimuth(), next_segment.azimuth()));
                            if (direction > (180.0 - reverse_tolerance)) {
                                // reverse movement detected
                                const auto position = std::distance(segments.begin(), next_it);
                                const auto size = segments.size();
                                _simplify(segments, index, next_it, tolerance);
                                const auto diff = size - segments.size();
                                it = std::next(segments.begin(), position - diff);
                                index = it;
                            }
                        }
                    }
                }
                // remaining
                _simplify(segments, index, segments.end(), tolerance);
            }
        }

        friend bool operator==(const rich_segment &left, const rich_segment &right) {
            return geometry::equals_points(left.first(), right.first()) and
                   geometry::equals_points(left.second(), right.second());
        }

        friend bool operator!=(const rich_segment &left, const rich_segment &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const rich_segment rich_segment) {
            return out << rich_segment.str();
        }

        friend class boost::serialization::access;

        template<typename Archive, typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _segment;
        }

    protected:
        segment_type _segment;

        template<typename RichSegment>
        static void _simplify(std::list<RichSegment> &segments, typename std::list<RichSegment>::iterator begin,
                              typename std::list<RichSegment>::iterator end, double tolerance) {
            if (std::distance(begin, end) < 2) {
                return;
            }

            typename std::list<RichSegment>::iterator max_it;
            using distance_type = typename data<point_type>::distance_type;
            distance_type max_distance = geometry::default_float_type<distance_type>::v0;

            RichSegment complete_segment{segment_type{(*begin).first(), (*std::prev(end)).second()}};
//        std::cout << complete_segment.wkt() << std::endl;

            for (auto it = std::next(begin); it != end; it++) {
                const RichSegment &segment = *it;
                const auto distance = geometry::distance(segment.segment().first, complete_segment.segment());

                if (distance > max_distance) {
                    max_distance = distance;
                    max_it = it;
                }
            }

            if (max_distance > tolerance) {
                _simplify(segments, begin, max_it, tolerance);
                _simplify(segments, max_it, end, tolerance);
            } else {
                auto last_it = segments.erase(begin, end);
                segments.insert(last_it, complete_segment);
            }
        }

    };

    template<typename Segment>
    class eager_rich_segment : public rich_segment<Segment>, public base_rich_segment<Segment> {

    public:
        using segment_type = Segment;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;

        template<typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        eager_rich_segment()
                : rich_segment<segment_type>{}, base_rich_segment<segment_type>{} {}

        explicit eager_rich_segment(segment_type segment)
                : rich_segment<segment_type>{std::move(segment)}, base_rich_segment<segment_type>{} {
            _compute();
        }

        eager_rich_segment(segment_type segment, length_type length, angle_type azimuth)
                : base_rich_segment<segment_type>{length, azimuth}, rich_segment<segment_type>{std::move(segment)} {
            if (geometry::equals_points(this->first(), this->second())) {
                this->_has_length = false;
                this->_has_azimuth = false;
            }
        }

        ~eager_rich_segment() = default;

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_constructible_v<SegmentT>, bool> = true>
        eager_rich_segment(const eager_rich_segment &other)
                : rich_segment<segment_type>{other}, base_rich_segment<segment_type>{other} {}

        template<typename SegmentT>
        eager_rich_segment(const eager_rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other}, base_rich_segment<segment_type>{other} {}

        template<typename SegmentT>
        eager_rich_segment(const rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other} {
            _compute();
        }

        template<typename SegmentT>
        eager_rich_segment(const lazy_rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other} {
            _transfer(other);
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_constructible_v<SegmentT>, bool> = true>
        eager_rich_segment(eager_rich_segment &&other) noexcept
                : rich_segment<segment_type>{std::move(other)}, base_rich_segment<segment_type>{std::move(other)} {}

        template<typename SegmentT>
        eager_rich_segment(eager_rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)}, base_rich_segment<segment_type>{std::move(other)} {}

        template<typename SegmentT>
        eager_rich_segment(rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)} {
            _compute();
        }

        template<typename SegmentT>
        eager_rich_segment(lazy_rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)} {
            _transfer(other);
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_assignable_v<SegmentT>, bool> = true>
        eager_rich_segment &operator=(const eager_rich_segment &other) {
            rich_segment<segment_type>::operator=(other);
            base_rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(const eager_rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            base_rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(const rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            _compute();
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(const lazy_rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            _transfer(other);
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_assignable_v<SegmentT>, bool> = true>
        eager_rich_segment &operator=(eager_rich_segment &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            base_rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(eager_rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            base_rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            _compute();
            return *this;
        }

        template<typename SegmentT>
        eager_rich_segment &operator=(lazy_rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            _transfer(other);
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            return this->_has_length;
        }

        [[nodiscard]] length_type length() const {
            return this->_length;
        }

        [[nodiscard]] bool has_azimuth() const {
            return this->_has_azimuth;
        }

        [[nodiscard]] angle_type azimuth() const {
            return this->_azimuth;
        }

        [[nodiscard]] std::string str() const {
            std::string str = "EagerRichSegment(";
            str.append(this->wkt());
            if (this->has_length()) {
                str.append(",");
                str.append(std::to_string(this->length()));
            }
            if (this->has_azimuth()) {
                str.append(",");
                str.append(std::to_string(this->azimuth()));
            }
            str.append(")");
            return str;
        }

        friend class boost::serialization::access;

        template<typename Archive, typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_segment<segment_type>>(*this);
            ar & boost::serialization::base_object<base_rich_segment<segment_type>>(*this);
        }

    protected:
        void _compute_length() const {
            this->_has_length = false;
            if (not geometry::equals_points(this->first(), this->second())) {
                this->_length = geometry::length_segment(this->_segment);
                this->_has_length = true;
            }
        }

        void _compute_azimuth() const {
            this->_has_azimuth = false;
            if (not geometry::equals_points(this->first(), this->second())) {
                this->_azimuth = geometry::azimuth_segment_deg(this->_segment);
                this->_has_azimuth = true;
            }
        }

        void _compute() const {
            _compute_length();
            _compute_azimuth();
        }

        template<typename SegmentT>
        void _transfer(const lazy_base_rich_segment<SegmentT> &other) {
            bool transferred = false;
            if (other.computed_length().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_length().load(std::memory_order_relaxed)) {
                    this->_length = other.length();
                    this->_has_length = other.has_length();
                    transferred = true;
                }
            }

            if (not transferred) {
                _compute_length();
            }

            transferred = false;
            if (other.computed_azimuth().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_azimuth().load(std::memory_order_relaxed)) {
                    this->_azimuth = other.azimuth();
                    this->_has_azimuth = other.has_azimuth();
                    transferred = true;
                }
            }

            if (not transferred) {
                _compute_azimuth();
            }
        }

    };

    template<typename Segment>
    class lazy_rich_segment : public rich_segment<Segment>, public lazy_base_rich_segment<Segment> {

    public:
        using segment_type = Segment;
        using length_type = typename data<segment_type>::length_type;
        using angle_type = typename data<segment_type>::angle_type;

        template<typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        lazy_rich_segment()
                : rich_segment<segment_type>{}, lazy_base_rich_segment<segment_type>{} {}

        explicit lazy_rich_segment(segment_type segment)
                : rich_segment<segment_type>{std::move(segment)}, lazy_base_rich_segment<segment_type>{} {}

        ~lazy_rich_segment() = default;

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_constructible_v<SegmentT>, bool> = true>
        lazy_rich_segment(const lazy_rich_segment &other)
                : rich_segment<segment_type>{other}, lazy_base_rich_segment<segment_type>{other} {}

        template<typename SegmentT>
        lazy_rich_segment(const lazy_rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other}, lazy_base_rich_segment<segment_type>{other} {}

        template<typename SegmentT>
        lazy_rich_segment(const rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other}, lazy_base_rich_segment<segment_type>{} {}

        template<typename SegmentT>
        lazy_rich_segment(const eager_rich_segment<SegmentT> &other)
                : rich_segment<segment_type>{other}, lazy_base_rich_segment<segment_type>{other} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_constructible_v<SegmentT>, bool> = true>
        lazy_rich_segment(lazy_rich_segment &&other) noexcept
                : rich_segment<segment_type>{std::move(other)},
                  lazy_base_rich_segment<segment_type>{std::move(other)} {}

        template<typename SegmentT>
        lazy_rich_segment(lazy_rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)},
                  lazy_base_rich_segment<segment_type>{std::move(other)} {}

        template<typename SegmentT>
        lazy_rich_segment(rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)}, lazy_base_rich_segment<segment_type>{} {}

        template<typename SegmentT>
        lazy_rich_segment(eager_rich_segment<SegmentT> &&other) noexcept
                : rich_segment<segment_type>{std::move(other)},
                  lazy_base_rich_segment<segment_type>{std::move(other)} {}

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_copy_assignable_v<SegmentT>, bool> = true>
        lazy_rich_segment &operator=(const lazy_rich_segment &other) {
            rich_segment<segment_type>::operator=(other);
            lazy_base_rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(const lazy_rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            lazy_base_rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(const rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(const eager_rich_segment<SegmentT> &other) {
            rich_segment<segment_type>::operator=(other);
            lazy_base_rich_segment<segment_type>::operator=(other);
            return *this;
        }

        template<typename SegmentT = segment_type,
                std::enable_if_t<std::is_move_assignable_v<SegmentT>, bool> = true>
        lazy_rich_segment &operator=(lazy_rich_segment &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            lazy_base_rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(lazy_rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            lazy_base_rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename SegmentT>
        lazy_rich_segment &operator=(eager_rich_segment<SegmentT> &&other) noexcept {
            rich_segment<segment_type>::operator=(std::move(other));
            lazy_base_rich_segment<segment_type>::operator=(std::move(other));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            _compute_length();
            return this->_has_length;
        }

        [[nodiscard]] length_type length() const {
            _compute_length();
            // could be invalid if no length was calculated, always use has_length before
            return this->_length;
        }

        [[nodiscard]] bool has_azimuth() const {
            _compute_azimuth();
            return this->_has_azimuth;
        }

        [[nodiscard]] angle_type azimuth() const {
            _compute_azimuth();
            // could be invalid if no azimuth could be calculated, always use has_azimuth before
            return this->_azimuth;
        }

        [[nodiscard]] std::string str() const {
            std::string str = "LazyRichSegment(";
            str.append(this->wkt());
            if (this->has_length()) {
                str.append(",");
                str.append(std::to_string(this->length()));
            }
            if (this->has_azimuth()) {
                str.append(",");
                str.append(std::to_string(this->azimuth()));
            }
            str.append(")");
            return str;
        }

        friend class boost::serialization::access;

        template<typename Archive, typename SegmentT = segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<SegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_segment<segment_type>>(*this);
        }

    private:
        void _compute_length() const {
            if (not this->_computed_length.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(this->_mutex);
                if (not this->_computed_length.load(std::memory_order_relaxed)) {
                    this->_has_length = false;
                    if (not geometry::equals_points(this->first(), this->second())) {
                        this->_length = geometry::length_segment(this->segment());
                        this->_has_length = true;
                    }
                    this->_computed_length.store(true, std::memory_order_release);
                }
            }
        }

        void _compute_azimuth() const {
            if (not this->_computed_azimuth.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(this->_mutex);
                if (not this->_computed_azimuth.load(std::memory_order_relaxed)) {
                    this->_has_azimuth = false;
                    if (not geometry::equals_points(this->first(), this->second())) {
                        this->_azimuth = geometry::azimuth_segment_deg(this->segment());
                        this->_has_azimuth = true;
                    }
                    this->_computed_azimuth.store(true, std::memory_order_release);
                }
            }
        }

    };

    template<typename Point>
    using rich_segment_type = rich_segment<typename models<Point>::segment_type>;

    template<typename Point>
    using rich_referring_segment_type = rich_segment<typename models<Point>::referring_segment_type>;

    template<typename Point>
    using rich_pointing_segment_type = rich_segment<typename models<Point>::pointing_segment_type>;

    template<typename Point>
    using eager_rich_segment_type = eager_rich_segment<typename models<Point>::segment_type>;

    template<typename Point>
    using eager_rich_referring_segment_type = eager_rich_segment<typename models<Point>::referring_segment_type>;

    template<typename Point>
    using eager_rich_pointing_segment_type = eager_rich_segment<typename models<Point>::pointing_segment_type>;

    template<typename Point>
    using lazy_rich_segment_type = lazy_rich_segment<typename models<Point>::segment_type>;

    template<typename Point>
    using lazy_rich_referring_segment_type = lazy_rich_segment<typename models<Point>::referring_segment_type>;

    template<typename Point>
    using lazy_rich_pointing_segment_type = lazy_rich_segment<typename models<Point>::pointing_segment_type>;

}

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::segment<Point> &segment, const unsigned int version) {
        ar & boost::serialization::make_nvp("point", static_cast<std::pair<Point, Point> &>(segment));
    }

}

#endif //MAP_MATCHING_2_RICH_SEGMENT_HPP
