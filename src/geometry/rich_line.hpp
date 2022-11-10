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

#ifndef MAP_MATCHING_2_RICH_LINE_HPP
#define MAP_MATCHING_2_RICH_LINE_HPP

#include <vector>
#include <iterator>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "rich_segment.hpp"

namespace map_matching_2::geometry {

    struct rich_line_merge_exception : public std::exception {
    };

    template<typename RichSegment>
    class lazy_base_rich_line;

    template<typename RichSegment>
    class base_rich_line {

    public:
        using rich_segment_type = RichSegment;
        using length_type = typename rich_segment_traits<rich_segment_type>::length_type;
        using angle_type = typename rich_segment_traits<rich_segment_type>::angle_type;

        ~base_rich_line() = default;

        base_rich_line(const base_rich_line &other) {
            _transfer(other);
        }

        template<typename RichSegmentT>
        base_rich_line(const base_rich_line<RichSegmentT> &other) {
            _transfer(other);
        }

        template<typename RichSegmentT>
        base_rich_line(const lazy_base_rich_line<RichSegmentT> &other) {
            _transfer(other);
        }

        base_rich_line(base_rich_line &&other) {
            _transfer(other);
        }

        template<typename RichSegmentT>
        base_rich_line(base_rich_line<RichSegmentT> &&other) noexcept {
            _transfer(other);
        }

        template<typename RichSegmentT>
        base_rich_line(lazy_base_rich_line<RichSegmentT> &&other) noexcept {
            _transfer(other);
        }

        base_rich_line &operator=(const base_rich_line &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        base_rich_line &operator=(const base_rich_line<RichSegmentT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        base_rich_line &operator=(const lazy_base_rich_line<RichSegmentT> &other) {
            _transfer(other);
            return *this;
        }

        base_rich_line &operator=(base_rich_line &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        base_rich_line &operator=(base_rich_line<RichSegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        base_rich_line &operator=(lazy_base_rich_line<RichSegmentT> &&other) noexcept {
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

        [[nodiscard]] bool has_directions() const {
            return _has_directions;
        }

        [[nodiscard]] angle_type directions() const {
            return _directions;
        }

        [[nodiscard]] angle_type absolute_directions() const {
            return _absolute_directions;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & _has_length;
            ar & _has_azimuth;
            ar & _has_directions;
            ar & _length;
            ar & _azimuth;
            ar & _directions;
            ar & _absolute_directions;
        }

    protected:
        mutable length_type _length;
        mutable angle_type _azimuth, _directions, _absolute_directions;
        mutable bool _has_length, _has_azimuth, _has_directions;

        base_rich_line()
                : _has_length{false}, _has_azimuth{false}, _has_directions{false} {}

        base_rich_line(length_type length, angle_type azimuth, angle_type directions, angle_type absolute_directions)
                : _length{length}, _has_length{true}, _has_azimuth{true}, _azimuth{azimuth}, _has_directions{true},
                  _directions{directions}, _absolute_directions{absolute_directions} {}

        template<typename RichSegmentT>
        void _transfer(const base_rich_line<RichSegmentT> &other) {
            if (other.has_length()) {
                this->_has_length = other.has_length();
                this->_length = other.length();
            }
            if (other.has_azimuth()) {
                this->_has_azimuth = other.has_azimuth();
                this->_azimuth = other.azimuth();
            }
            if (other.has_directions()) {
                this->_has_directions = other.has_directions();
                this->_directions = other.directions();
                this->_absolute_directions = other.absolute_directions();
            }
        }

    };

    template<typename RichSegment>
    class lazy_base_rich_line : public base_rich_line<RichSegment> {

    public:
        using rich_segment_type = RichSegment;

        ~lazy_base_rich_line() = default;

        lazy_base_rich_line(const lazy_base_rich_line &other)
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        template<typename RichSegmentT>
        lazy_base_rich_line(const lazy_base_rich_line<RichSegmentT> &other)
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        template<typename RichSegmentT>
        lazy_base_rich_line(const base_rich_line<RichSegmentT> &other)
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        lazy_base_rich_line(lazy_base_rich_line &&other) noexcept
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        template<typename RichSegmentT>
        lazy_base_rich_line(lazy_base_rich_line<RichSegmentT> &&other) noexcept
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        template<typename RichSegmentT>
        lazy_base_rich_line(base_rich_line<RichSegmentT> &&other) noexcept
                : base_rich_line<rich_segment_type>{} {
            _transfer(other);
        }

        lazy_base_rich_line &operator=(const lazy_base_rich_line &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_base_rich_line &operator=(const lazy_base_rich_line<RichSegmentT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_base_rich_line &operator=(const base_rich_line<RichSegmentT> &other) {
            _transfer(other);
            return *this;
        }

        lazy_base_rich_line &operator=(lazy_base_rich_line &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_base_rich_line &operator=(lazy_base_rich_line<RichSegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_base_rich_line &operator=(base_rich_line<RichSegmentT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        [[nodiscard]] std::atomic<bool> &computed_length() const {
            return _computed_length;
        }

        [[nodiscard]] std::atomic<bool> &computed_azimuth() const {
            return _computed_azimuth;
        }

        [[nodiscard]] std::atomic<bool> &computed_directions() const {
            return _computed_directions;
        }

        [[nodiscard]] std::mutex &mutex() const {
            return _mutex;
        }

    protected:
        mutable std::atomic<bool> _computed_length, _computed_azimuth, _computed_directions;
        mutable std::mutex _mutex;

        lazy_base_rich_line()
                : base_rich_line<rich_segment_type>{}, _computed_length{false}, _computed_azimuth{false},
                  _computed_directions{false} {}

        template<typename RichSegmentT>
        void _transfer(const lazy_base_rich_line<RichSegmentT> &other) {
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

            _computed_directions.store(false, std::memory_order_release);
            if (other.computed_directions().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(_mutex);
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_directions().load(std::memory_order_relaxed)) {
                    this->_directions = other.directions();
                    this->_absolute_directions = other.absolute_directions();
                    this->_has_directions = other.has_directions();
                    _computed_directions.store(true, std::memory_order_release);
                }
            }
        }

        template<typename RichSegmentT>
        void _transfer(const base_rich_line<RichSegmentT> &other) {
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

            _computed_directions.store(false, std::memory_order_release);
            if (other.has_directions()) {
                std::lock_guard<std::mutex> lock(_mutex);
                if (other.has_directions()) {
                    this->_directions = other.directions();
                    this->_absolute_directions = other.absolute_directions();
                    this->_has_directions = other.has_directions();
                    _computed_directions.store(true, std::memory_order_release);
                }
            }
        }

    };

    template<typename RichSegment>
    class rich_line_iterator {

    public:
        using rich_segment_type = RichSegment;
        using point_type = typename rich_segment_traits<rich_segment_type>::point_type;

        using iterator_category = std::random_access_iterator_tag;
        using difference_type = std::size_t;
        using value_type = point_type;
        using pointer = const point_type *;
        using reference = const point_type &;

        rich_line_iterator(const std::vector<rich_segment_type> &rich_segments, std::size_t i)
                : _rich_segments{rich_segments}, _i{i} {}

        reference operator*() const {
            return at(_i);
        }

        pointer operator->() {
            return &at(_i);
        }

        rich_line_iterator &operator++() {
            _i++;
            return *this;
        }

        rich_line_iterator &operator--() {
            _i--;
            return *this;
        }

        rich_line_iterator operator++(int) {
            return rich_line_iterator{_rich_segments, ++_i};
        }

        rich_line_iterator operator--(int) {
            return rich_line_iterator{_rich_segments, --_i};
        }

        rich_line_iterator &operator+=(difference_type n) {
            _i += n;
            return *this;
        }

        rich_line_iterator &operator-=(difference_type n) {
            _i -= n;
            return *this;
        }

        friend bool operator==(const rich_line_iterator &left, const rich_line_iterator &right) {
            return &left._rich_segments == &right._rich_segments and left._i == right._i;
        }

        friend bool operator!=(const rich_line_iterator &left, const rich_line_iterator &right) {
            return not(left == right);
        }

        friend difference_type operator-(const rich_line_iterator &left, const rich_line_iterator &right) {
            return (&left._rich_segments - &right._rich_segments) + (left._i - right._i);
        }

    protected:
        const std::vector<rich_segment_type> &_rich_segments;
        std::size_t _i;

        [[nodiscard]] const point_type &at(const std::size_t i) const {
            if (i == 0 || i < _rich_segments.size()) {
                return _rich_segments.at(i).first();
            } else {
                return _rich_segments.at(i - 1).second();
            }
        }

    };

    template<typename RichSegment>
    class eager_rich_line;

    template<typename RichSegment>
    class lazy_rich_line;

    template<typename RichSegment>
    class rich_line {

    public:
        using rich_segment_type = RichSegment;
        using segment_type = typename rich_segment_traits<rich_segment_type>::segment_type;
        using line_type = typename rich_segment_traits<rich_segment_type>::line_type;
        using point_type = typename rich_segment_traits<rich_segment_type>::point_type;
        using distance_type = typename rich_segment_traits<rich_segment_type>::distance_type;
        using length_type = typename rich_segment_traits<rich_segment_type>::length_type;
        using angle_type = typename rich_segment_traits<rich_segment_type>::angle_type;
        using const_iterator = rich_line_iterator<rich_segment_type>;

        template<typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        rich_line()
                : _rich_segments{} {}

        explicit rich_line(const line_type &line)
                : _rich_segments{line2rich_segments<rich_segment_type, std::vector>(line)} {}

        explicit rich_line(std::vector<rich_segment_type> rich_segments)
                : _rich_segments{std::move(rich_segments)} {}

        template<typename RichSegmentT>
        explicit rich_line(std::vector<RichSegmentT> rich_segments)
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(std::move(rich_segments))} {}

        ~rich_line() = default;

        rich_line(const rich_line &other)
                : _rich_segments{other.rich_segments()} {}

        template<typename RichSegmentT>
        rich_line(const rich_line<RichSegmentT> &other)
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments())} {}

        template<typename RichSegmentT>
        rich_line(const eager_rich_line<RichSegmentT> &other)
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments())} {}

        template<typename RichSegmentT>
        rich_line(const lazy_rich_line<RichSegmentT> &other)
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments())} {}

        rich_line(rich_line &&other) noexcept
                : _rich_segments{std::move(const_cast<std::vector<rich_segment_type> &>(other.rich_segments()))} {}

        template<typename RichSegmentT>
        rich_line(rich_line<RichSegmentT> &&other) noexcept
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(
                std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())))} {}

        template<typename RichSegmentT>
        rich_line(eager_rich_line<RichSegmentT> &&other) noexcept
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(
                std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())))} {}

        template<typename RichSegmentT>
        rich_line(lazy_rich_line<RichSegmentT> &&other) noexcept
                : _rich_segments{convert_rich_types<rich_segment_type, RichSegmentT>(
                std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())))} {}

        rich_line &operator=(const rich_line &other) {
            _rich_segments = other.rich_segments();
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(const rich_line<RichSegmentT> &other) {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments());
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(const eager_rich_line<RichSegmentT> &other) {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments());
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(const lazy_rich_line<RichSegmentT> &other) {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(other.rich_segments());
            return *this;
        }

        rich_line &operator=(rich_line &&other) noexcept {
            _rich_segments = std::move(const_cast<std::vector<rich_segment_type> &>(other.rich_segments()));
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(rich_line<RichSegmentT> &&other) noexcept {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(
                    std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())));
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(eager_rich_line<RichSegmentT> &&other) noexcept {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(
                    std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())));
            return *this;
        }

        template<typename RichSegmentT>
        rich_line &operator=(lazy_rich_line<RichSegmentT> &&other) noexcept {
            _rich_segments = convert_rich_types<rich_segment_type, RichSegmentT>(
                    std::move(const_cast<std::vector<RichSegmentT> &>(other.rich_segments())));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            if (not this->_rich_segments.empty()) {
                for (const auto &rich_segment: this->_rich_segments) {
                    if (rich_segment.has_length()) {
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] length_type length() const {
            if (not this->_rich_segments.empty()) {
                length_type length = default_float_type<length_type>::v0;
                for (const auto &rich_segment: this->_rich_segments) {
                    if (rich_segment.has_length()) {
                        length += rich_segment.length();
                    }
                }
                return length;
            }

            throw std::runtime_error("line has no length");
        }

        [[nodiscard]] bool has_azimuth() const {
            if (not this->_rich_segments.empty()) {
                const auto &start = this->_rich_segments.front().first();
                const auto &end = this->_rich_segments.back().second();
                return not geometry::equals_points(start, end);
            }
            return false;
        }

        [[nodiscard]] angle_type azimuth() const {
            if (not this->_rich_segments.empty()) {
                const auto &start = this->_rich_segments.front().first();
                const auto &end = this->_rich_segments.back().second();
                if (not geometry::equals_points(start, end)) {
                    return geometry::azimuth_deg(start, end);
                }
            }

            throw std::runtime_error("line has no azimuth");
        }

        [[nodiscard]] bool has_directions() const {
            if (this->_rich_segments.size() >= 2) {
                for (std::size_t i = 0; i < this->_rich_segments.size() - 1; ++i) {
                    angle_type segment_direction = this->segments_direction(i);
                    if (not std::isnan(segment_direction)) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] angle_type directions() const {
            if (this->_rich_segments.size() >= 2) {
                angle_type directions = default_float_type<angle_type>::v0;
                for (std::size_t i = 0; i < this->_rich_segments.size() - 1; ++i) {
                    angle_type segment_direction = this->segments_direction(i);
                    if (not std::isnan(segment_direction)) {
                        directions += segment_direction;
                    }
                }
                return directions;
            }

            throw std::runtime_error("line has no directions");
        }

        [[nodiscard]] angle_type absolute_directions() const {
            if (this->_rich_segments.size() >= 2) {
                angle_type absolute_directions = default_float_type<angle_type>::v0;
                for (std::size_t i = 0; i < this->_rich_segments.size() - 1; ++i) {
                    angle_type segment_direction = this->segments_direction(i);
                    if (not std::isnan(segment_direction)) {
                        absolute_directions += std::fabs(segment_direction);
                    }
                }
                return absolute_directions;
            }

            throw std::runtime_error("line has no absolute directions");
        }

        [[nodiscard]] const std::vector<rich_segment_type> &rich_segments() const {
            return _rich_segments;
        }

        [[nodiscard]] const point_type &at(const std::size_t i) const {
            if (i < _rich_segments.size()) {
                return _rich_segments.at(i).first();
            } else if (i > 0) {
                return _rich_segments.at(i - 1).second();
            }

            throw std::out_of_range("size: " + std::to_string(_rich_segments.size()) + ", i: " + std::to_string(i));
        }

        [[nodiscard]] const point_type &front() const {
            return _rich_segments.front().first();
        }

        [[nodiscard]] const point_type &back() const {
            return _rich_segments.back().second();
        }

        [[nodiscard]] const_iterator cbegin() const {
            return const_iterator{_rich_segments, 0};
        }

        [[nodiscard]] const_iterator cend() const {
            return const_iterator{_rich_segments, _rich_segments.size() + 1};
        }

        [[nodiscard]] line_type line() const {
            return rich_segments2line<rich_segment_type>(_rich_segments);
        }

        [[nodiscard]] bool empty() const {
            return _rich_segments.empty();
        }

        [[nodiscard]] std::size_t size() const {
            return _rich_segments.empty() ? 0 : _rich_segments.size() + 1;
        }

        [[nodiscard]] angle_type segments_direction(std::size_t index) const {
            if (index >= _rich_segments.size() - 1) {
                throw std::out_of_range("index is out of range");
            }

            const auto &a = _rich_segments[index];
            const auto &b = _rich_segments[index + 1];
            if (geometry::equals_points(a.second(), b.first())) {
                if (a.has_azimuth() and b.has_azimuth()) {
                    return direction_deg(a.azimuth(), b.azimuth());
                }
            }

            return std::numeric_limits<angle_type>::signaling_NaN();
        }

        [[nodiscard]] std::string wkt() const {
            return geometry::to_wkt(line());
        }

        [[nodiscard]] std::string str() const {
            std::string str = "RichLine(";
            str.append(wkt());
            str.append(")");
            return str;
        }

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            std::list<rich_segment_type> rich_segments_list;
            std::move(_rich_segments.begin(), _rich_segments.end(), std::back_inserter(rich_segments_list));
            _rich_segments.clear();

            rich_segment_type::simplify(rich_segments_list, retain_reversals, tolerance, reverse_tolerance);

            _rich_segments.reserve(rich_segments_list.size());
            std::move(rich_segments_list.begin(), rich_segments_list.end(), std::back_inserter(_rich_segments));
        }

        template<typename Network, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            using index_key_type = std::pair<point_type, typename std::list<point_type>::iterator>;
            using index_type = boost::geometry::index::rtree<index_key_type, boost::geometry::index::rstar<16>>;

            std::list<point_type> points;
            for (auto it = _rich_segments.cbegin(); it != _rich_segments.cend(); it++) {
                if (it == _rich_segments.cbegin()) {
                    points.emplace_back(it->first());
                }
                points.emplace_back(it->second());
            }

            std::vector<index_key_type> index_points;
            index_points.reserve(points.size());
            for (auto it = points.begin(); it != points.end(); it++) {
                index_points.emplace_back(index_key_type{*it, it});
            }

            index_type points_index{index_points};

            auto it = points.begin();
            while (it != points.end()) {
                const point_type &point = *it;

                double current_tolerance = tolerance;
                if (adaptive and network != nullptr) {
                    const auto edge_result = network->query_segments_unique(boost::geometry::index::nearest(point, 1));
                    if (not edge_result.empty()) {
                        const auto &edge = network->edge(*edge_result.cbegin());
                        auto distance = geometry::distance(point, edge.line());
                        current_tolerance = std::max(tolerance, distance * 0.5);
                    }
                }

                const auto buffer = geometry::buffer_box(point, current_tolerance);
                std::list<index_key_type> results;
                points_index.query(boost::geometry::index::intersects(buffer), std::back_inserter(results));

                // remove results outside current tolerance
                for (auto results_it = results.begin(); results_it != results.end();) {
                    if (geometry::point_distance(point, results_it->first) > current_tolerance) {
                        results_it = results.erase(results_it);
                    } else {
                        results_it++;
                    }
                }

                // only retain connected points
                if (results.size() > 1) {
//                std::cout << geometry::to_wkt(buffer) << std::endl;

                    // only continue if we found at least one other point next to our current point
                    std::vector<index_key_type> connected_results;
                    connected_results.reserve(results.size());
                    auto current_it = it;
                    bool search = true;
                    auto result_it = results.begin();
                    while (search) {
                        auto &result = *result_it;
                        if (current_it == result.second) {
                            // found current point
                            connected_results.emplace_back(std::move(result));
                            results.erase(result_it);
                            // start again
                            result_it = results.begin();
                            current_it++;
                        } else {
                            result_it++;
                        }

                        if (result_it == results.end() or current_it == points.end()) {
                            search = false;
                        }
                    }

                    if (not connected_results.empty()) {
                        std::vector<point_type> result_points;
                        result_points.reserve(connected_results.size());
                        for (const auto &result: connected_results) {
                            result_points.emplace_back(result.first);
                        }

                        point_type median_point = geometry::median_point(result_points);

                        for (const auto &result: connected_results) {
                            points.erase(result.second);
                            points_index.remove(result);
                        }

                        it = current_it;
                        if (it != points.end()) {
                            points.insert(it, median_point);
                        } else {
                            points.emplace_back(median_point);
                        }
                    }
                } else {
                    it++;
                }
            }

            _rich_segments = points2rich_segments<rich_segment_type, std::vector>(points);
        }

        template<typename RichLine>
        [[nodiscard]] static RichLine merge(const std::vector<std::reference_wrapper<RichLine>> &rich_lines) {
            using rich_line_type = RichLine;
            using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;

            // calculate new size
            std::size_t new_size = 0;
            for (const rich_line_type &rich_line: rich_lines) {
                new_size += rich_line._rich_segments.size();
            }

            std::vector<rich_segment_type> new_rich_segments;
            new_rich_segments.reserve(new_size);

            for (rich_line_type &next_rich_line: rich_lines) {
                auto &next_rich_segments = next_rich_line._rich_segments;
                if (not next_rich_segments.empty()) {
                    if (not new_rich_segments.empty()) {
                        const auto &prev_point = new_rich_segments.back().second();
                        const auto &next_point = next_rich_segments.front().first();
                        if (not geometry::equals_points(prev_point, next_point)) {
//                    std::clog << "Prev: " << geometry::to_wkt(prev_point) << "\n"
//                              << "Next: " << geometry::to_wkt(next_point) << std::endl;
                            throw rich_line_merge_exception{};
                        }
                    }

                    std::move(next_rich_segments.begin(), next_rich_segments.end(),
                              std::back_inserter(new_rich_segments));
                }
            }

            return rich_line_type{std::move(new_rich_segments)};
        }

        friend bool operator==(const rich_line &left, const rich_line &right) {
            // if size of segments is not equal, lines cannot be equal
            if (left._rich_segments.size() != right._rich_segments.size()) {
                return false;
            }

            // if any segments is not equal, lines cannot be equal
            for (std::size_t i = 0; i < left._rich_segments.size(); ++i) {
                if (left._rich_segments[i] != right._rich_segments[i]) {
                    return false;
                }
            }

            // if all segments are equal, lines are equal
            return true;
        }

        friend bool operator!=(const rich_line &left, const rich_line &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const rich_line &rich_line) {
            return out << rich_line.str();
        }

        friend class boost::serialization::access;

        template<typename Archive, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void save(Archive &ar, const unsigned int version) const {
            line_type _line = this->line();
            ar << _line;
        }

        template<typename Archive, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void load(Archive &ar, const unsigned int version) {
            line_type _line;
            ar >> _line;
            _rich_segments = line2rich_segments<rich_segment_type, std::vector>(_line);
        }

        template<typename Archive, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            boost::serialization::split_member(ar, *this, version);
        }

    protected:
        std::vector<rich_segment_type> _rich_segments;

    };

    template<typename RichSegment>
    class eager_rich_line : public rich_line<RichSegment>, public base_rich_line<RichSegment> {

    public:
        using rich_segment_type = RichSegment;
        using line_type = typename rich_segment_traits<rich_segment_type>::line_type;
        using length_type = typename rich_segment_traits<rich_segment_type>::length_type;
        using angle_type = typename rich_segment_traits<rich_segment_type>::angle_type;
        using const_iterator = rich_line_iterator<rich_segment_type>;

        template<typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        eager_rich_line()
                : rich_line<rich_segment_type>{}, base_rich_line<rich_segment_type>{} {}

        explicit eager_rich_line(const line_type &line)
                : rich_line<rich_segment_type>{line}, base_rich_line<rich_segment_type>{} {
            _compute();
        }

        explicit eager_rich_line(std::vector<rich_segment_type> rich_segments)
                : rich_line<rich_segment_type>{std::move(rich_segments)}, base_rich_line<rich_segment_type>{} {
            _compute();
        }

        template<typename RichSegmentT>
        explicit eager_rich_line(std::vector<RichSegmentT> rich_segments)
                : rich_line<rich_segment_type>{std::move(rich_segments)}, base_rich_line<rich_segment_type>{} {
            _compute();
        }

        ~eager_rich_line() = default;

        eager_rich_line(const eager_rich_line &other)
                : rich_line<rich_segment_type>{other}, base_rich_line<rich_segment_type>{other} {}

        template<typename RichSegmentT>
        eager_rich_line(const eager_rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other}, base_rich_line<rich_segment_type>{other} {}

        template<typename RichSegmentT>
        eager_rich_line(const rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other} {
            _compute();
        }

        template<typename RichSegmentT>
        eager_rich_line(const lazy_rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other} {
            _transfer(other);
        }

        eager_rich_line(eager_rich_line &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)}, base_rich_line<rich_segment_type>{std::move(other)} {}

        template<typename RichSegmentT>
        eager_rich_line(eager_rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)}, base_rich_line<rich_segment_type>{std::move(other)} {}

        template<typename RichSegmentT>
        eager_rich_line(rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)} {
            _compute();
        }

        template<typename RichSegmentT>
        eager_rich_line(lazy_rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)} {
            _transfer(other);
        }

        eager_rich_line &operator=(const eager_rich_line &other) {
            rich_line<rich_segment_type>::operator=(other);
            base_rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(const eager_rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            base_rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(const rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            _compute();
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(const lazy_rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            _transfer(other);
            return *this;
        }

        eager_rich_line &operator=(eager_rich_line &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            base_rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(eager_rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            base_rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            _compute();
            return *this;
        }

        template<typename RichSegmentT>
        eager_rich_line &operator=(lazy_rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
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

        [[nodiscard]] bool has_directions() const {
            return this->_has_directions;
        }

        [[nodiscard]] angle_type directions() const {
            return this->_directions;
        }

        [[nodiscard]] angle_type absolute_directions() const {
            return this->_absolute_directions;
        }

        [[nodiscard]] std::string str() const {
            std::string str = "EagerRichLine(";
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

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            rich_line<rich_segment_type>::simplify(retain_reversals, tolerance, reverse_tolerance);
            _compute();
        }

        template<typename Network, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            rich_line<rich_segment_type>::median_merge(tolerance, adaptive, network);
            _compute();
        }

        friend class boost::serialization::access;

        template<typename Archive, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_line<rich_segment_type>>(*this);
            ar & boost::serialization::base_object<base_rich_line<rich_segment_type>>(*this);
        }

    private:
        void _compute_length() const {
            this->_has_length = false;
            if (not this->_rich_segments.empty()) {
                this->_length = default_float_type<length_type>::v0;
                for (const auto &rich_segment: this->_rich_segments) {
                    if (rich_segment.has_length()) {
                        this->_length += rich_segment.length();
                        this->_has_length = true;
                    }
                }
            }
        }

        void _compute_azimuth() const {
            this->_has_azimuth = false;
            if (not this->_rich_segments.empty()) {
                const auto &start = this->_rich_segments.front().first();
                const auto &end = this->_rich_segments.back().second();
                if (not geometry::equals_points(start, end)) {
                    this->_azimuth = geometry::azimuth_deg(start, end);
                    this->_has_azimuth = true;
                }
            }
        }

        void _compute_directions() const {
            this->_has_directions = false;
            if (this->_rich_segments.size() >= 2) {
                this->_directions = default_float_type<angle_type>::v0;
                this->_absolute_directions = default_float_type<angle_type>::v0;
                for (std::size_t i = 0; i < this->_rich_segments.size() - 1; ++i) {
                    angle_type segment_direction = this->segments_direction(i);
                    if (not std::isnan(segment_direction)) {
                        this->_directions += segment_direction;
                        this->_absolute_directions += std::fabs(segment_direction);
                        this->_has_directions = true;
                    }
                }
            }
        }

        void _compute() const {
            _compute_length();
            _compute_azimuth();
            _compute_directions();
        }

        template<typename RichSegmentT>
        void _transfer(const lazy_base_rich_line<RichSegmentT> &other) {
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

            transferred = false;
            if (other.computed_directions().load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> other_lock(other.mutex());
                if (other.computed_directions().load(std::memory_order_relaxed)) {
                    this->_directions = other.directions();
                    this->_absolute_directions = other.absolute_directions();
                    this->_has_directions = other.has_directions();
                    transferred = true;
                }
            }

            if (not transferred) {
                _compute_directions();
            }
        }

    };

    template<typename RichSegment>
    class lazy_rich_line : public rich_line<RichSegment>, public lazy_base_rich_line<RichSegment> {

    public:
        using rich_segment_type = RichSegment;
        using segment_type = typename rich_segment_traits<rich_segment_type>::segment_type;
        using line_type = typename rich_segment_traits<rich_segment_type>::line_type;
        using length_type = typename rich_segment_traits<rich_segment_type>::length_type;
        using angle_type = typename rich_segment_traits<rich_segment_type>::angle_type;
        using const_iterator = rich_line_iterator<rich_segment_type>;

        template<typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        lazy_rich_line()
                : rich_line<rich_segment_type>{}, lazy_base_rich_line<rich_segment_type>{} {}

        explicit lazy_rich_line(const line_type &line)
                : rich_line<rich_segment_type>{line}, lazy_base_rich_line<rich_segment_type>{} {}

        explicit lazy_rich_line(std::vector<rich_segment_type> rich_segments)
                : rich_line<rich_segment_type>{std::move(rich_segments)}, lazy_base_rich_line<rich_segment_type>{} {}

        template<typename RichSegmentT>
        explicit lazy_rich_line(std::vector<RichSegmentT> rich_segments)
                : rich_line<rich_segment_type>{std::move(rich_segments)}, lazy_base_rich_line<rich_segment_type>{} {}

        ~lazy_rich_line() = default;

        lazy_rich_line(const lazy_rich_line &other)
                : rich_line<rich_segment_type>{other}, lazy_base_rich_line<rich_segment_type>{other} {}

        template<typename RichSegmentT>
        lazy_rich_line(const lazy_rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other}, lazy_base_rich_line<rich_segment_type>{other} {}

        template<typename RichSegmentT>
        lazy_rich_line(const rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other}, lazy_base_rich_line<rich_segment_type>{} {}

        template<typename RichSegmentT>
        lazy_rich_line(const eager_rich_line<RichSegmentT> &other)
                : rich_line<rich_segment_type>{other}, lazy_base_rich_line<rich_segment_type>{other} {}

        lazy_rich_line(lazy_rich_line &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)},
                  lazy_base_rich_line<rich_segment_type>{std::move(other)} {}

        template<typename RichSegmentT>
        lazy_rich_line(lazy_rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)},
                  lazy_base_rich_line<rich_segment_type>{std::move(other)} {}

        template<typename RichSegmentT>
        lazy_rich_line(rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)}, lazy_base_rich_line<rich_segment_type>{} {}

        template<typename RichSegmentT>
        lazy_rich_line(eager_rich_line<RichSegmentT> &&other) noexcept
                : rich_line<rich_segment_type>{std::move(other)},
                  lazy_base_rich_line<rich_segment_type>{std::move(other)} {}

        lazy_rich_line &operator=(const lazy_rich_line &other) {
            rich_line<rich_segment_type>::operator=(other);
            lazy_base_rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(const lazy_rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            lazy_base_rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(const rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(const eager_rich_line<RichSegmentT> &other) {
            rich_line<rich_segment_type>::operator=(other);
            lazy_base_rich_line<rich_segment_type>::operator=(other);
            return *this;
        }

        lazy_rich_line &operator=(lazy_rich_line &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            lazy_base_rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(lazy_rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            lazy_base_rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichSegmentT>
        lazy_rich_line &operator=(eager_rich_line<RichSegmentT> &&other) noexcept {
            rich_line<rich_segment_type>::operator=(std::move(other));
            lazy_base_rich_line<rich_segment_type>::operator=(std::move(other));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            _compute_length();
            return this->_has_length;
        }

        [[nodiscard]] length_type length() const {
            _compute_length();
            return this->_length;
        }

        [[nodiscard]] bool has_azimuth() const {
            _compute_azimuth();
            return this->_has_azimuth;
        }

        [[nodiscard]] angle_type azimuth() const {
            _compute_azimuth();
            return this->_azimuth;
        }

        [[nodiscard]] bool has_directions() const {
            _compute_directions();
            return this->_has_directions;
        }

        [[nodiscard]] angle_type directions() const {
            _compute_directions();
            return this->_directions;
        }

        [[nodiscard]] angle_type absolute_directions() const {
            _compute_directions();
            return this->_absolute_directions;
        }

        [[nodiscard]] std::string str() const {
            std::string str = "LazyRichLine(";
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

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            rich_line<rich_segment_type>::simplify(retain_reversals, tolerance, reverse_tolerance);
            this->_computed_length.store(false, std::memory_order_release);
            this->_computed_azimuth.store(false, std::memory_order_release);
            this->_computed_directions.store(false, std::memory_order_release);
        }

        template<typename Network, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            rich_line<rich_segment_type>::median_merge(tolerance, adaptive, network);
            this->_computed_length.store(false, std::memory_order_release);
            this->_computed_azimuth.store(false, std::memory_order_release);
            this->_computed_directions.store(false, std::memory_order_release);
        }

        friend class boost::serialization::access;

        template<typename Archive, typename RichSegmentT = rich_segment_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichSegmentT>>>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<rich_line<rich_segment_type>>(*this);
        }

    private:
        void _compute_length() const {
            if (not this->_computed_length.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(this->_mutex);
                if (not this->_computed_length.load(std::memory_order_relaxed)) {
                    this->_has_length = false;
                    if (not this->_rich_segments.empty()) {
                        this->_length = default_float_type<length_type>::v0;
                        for (const auto &rich_segment: this->_rich_segments) {
                            if (rich_segment.has_length()) {
                                this->_length += rich_segment.length();
                                this->_has_length = true;
                            }
                        }
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
                    if (not this->_rich_segments.empty()) {
                        const auto &start = this->_rich_segments.front().first();
                        const auto &end = this->_rich_segments.back().second();
                        if (not geometry::equals_points(start, end)) {
                            this->_azimuth = geometry::azimuth_deg(start, end);
                            this->_has_azimuth = true;
                        }
                    }
                    this->_computed_azimuth.store(true, std::memory_order_release);
                }
            }
        }

        void _compute_directions() const {
            if (not this->_computed_directions.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(this->_mutex);
                if (not this->_computed_directions.load(std::memory_order_relaxed)) {
                    this->_has_directions = false;
                    if (this->_rich_segments.size() >= 2) {
                        this->_directions = default_float_type<angle_type>::v0;
                        this->_absolute_directions = default_float_type<angle_type>::v0;
                        for (std::size_t i = 0; i < this->_rich_segments.size() - 1; ++i) {
                            angle_type segment_direction = this->segments_direction(i);
                            if (not std::isnan(segment_direction)) {
                                this->_directions += segment_direction;
                                this->_absolute_directions += std::fabs(segment_direction);
                                this->_has_directions = true;
                            }
                        }
                    }
                    this->_computed_directions.store(true, std::memory_order_release);
                }
            }
        }

    };

    template<typename Point>
    using rich_line_type = rich_line<rich_segment_type<Point>>;

    template<typename Point>
    using rich_referring_line_type = rich_line<rich_referring_segment_type<Point>>;

    template<typename Point>
    using rich_pointing_line_type = rich_line<rich_pointing_segment_type<Point>>;

    template<typename Point>
    using eager_rich_line_type = eager_rich_line<eager_rich_segment_type<Point>>;

    template<typename Point>
    using eager_rich_referring_line_type = eager_rich_line<eager_rich_referring_segment_type<Point>>;

    template<typename Point>
    using eager_rich_pointing_line_type = eager_rich_line<eager_rich_pointing_segment_type<Point>>;

    template<typename Point>
    using lazy_rich_line_type = lazy_rich_line<lazy_rich_segment_type<Point>>;

    template<typename Point>
    using lazy_rich_referring_line_type = lazy_rich_line<lazy_rich_referring_segment_type<Point>>;

    template<typename Point>
    using lazy_rich_pointing_line_type = lazy_rich_line<lazy_rich_pointing_segment_type<Point>>;

}

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::linestring<Point> &line, const unsigned int version) {
        ar & boost::serialization::make_nvp("line", static_cast<std::vector<Point> &>(line));
    }

}

#endif //MAP_MATCHING_2_RICH_LINE_HPP
