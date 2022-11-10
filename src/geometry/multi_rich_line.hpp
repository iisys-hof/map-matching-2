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

#ifndef MAP_MATCHING_2_MULTI_RICH_LINE_HPP
#define MAP_MATCHING_2_MULTI_RICH_LINE_HPP

#include "rich_line.hpp"

namespace map_matching_2::geometry {

    template<typename MultiRichLine>
    [[nodiscard]] static typename multi_rich_line_traits<MultiRichLine>::multi_line_type
    lines2multi_line(const std::vector<typename multi_rich_line_traits<MultiRichLine>::line_type> &lines) {
        using multi_line_type = typename multi_rich_line_traits<MultiRichLine>::multi_line_type;

        multi_line_type multi_line;
        multi_line.reserve(lines.size());
        for (const auto &line: lines) {
            if (not line.empty()) {
                multi_line.emplace_back(line);
            }
        }
        return multi_line;
    }

    template<typename MultiRichLine>
    [[nodiscard]] static typename multi_rich_line_traits<MultiRichLine>::multi_line_type
    rich_lines2multi_line(
            const std::vector<typename multi_rich_line_traits<MultiRichLine>::rich_line_type> &rich_lines) {
        using multi_line_type = typename multi_rich_line_traits<MultiRichLine>::multi_line_type;

        multi_line_type multi_line;
        multi_line.reserve(rich_lines.size());
        for (const auto &rich_line: rich_lines) {
            if (not rich_line.empty()) {
                multi_line.emplace_back(rich_line.line());
            }
        }
        return multi_line;
    }

    template<typename MultiRichLine,
            template<typename> class Container>
    [[nodiscard]] static Container<typename multi_rich_line_traits<MultiRichLine>::rich_line_type>
    multi_line2rich_lines(const typename multi_rich_line_traits<MultiRichLine>::multi_line_type &multi_lines) {
        using rich_line_type = typename multi_rich_line_traits<MultiRichLine>::rich_line_type;

        Container<rich_line_type> rich_lines;
        if (not multi_lines.empty()) {
            if constexpr (std::is_void_v<decltype(&Container<rich_line_type>::reserve)>) {
                rich_lines.reserve(multi_lines.size());
            }
            for (const auto &line: multi_lines) {
                if (not line.empty()) {
                    rich_lines.emplace_back(rich_line_type{line});
                }
            }
        }
        return rich_lines;
    }

    template<typename RichLine>
    class lazy_base_multi_rich_line;

    template<typename RichLine>
    class base_multi_rich_line {

    public:
        using rich_line_type = RichLine;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        ~base_multi_rich_line() = default;

        base_multi_rich_line(const base_multi_rich_line &other) {
            _transfer(other);
        }

        template<typename RichLineT>
        base_multi_rich_line(const base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
        }

        template<typename RichLineT>
        base_multi_rich_line(const lazy_base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
        }

        base_multi_rich_line(base_multi_rich_line &&other) {
            _transfer(other);
        }

        template<typename RichLineT>
        base_multi_rich_line(base_multi_rich_line<RichLineT> &&other) noexcept {
            _transfer(other);
        }

        template<typename RichLineT>
        base_multi_rich_line(lazy_base_multi_rich_line<RichLineT> &&other) noexcept {
            _transfer(other);
        }

        base_multi_rich_line &operator=(const base_multi_rich_line &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        base_multi_rich_line &operator=(const base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        base_multi_rich_line &operator=(const lazy_base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
            return *this;
        }

        base_multi_rich_line &operator=(base_multi_rich_line &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        base_multi_rich_line &operator=(base_multi_rich_line<RichLineT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        base_multi_rich_line &operator=(lazy_base_multi_rich_line<RichLineT> &&other) noexcept {
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

    protected:
        mutable length_type _length;
        mutable angle_type _azimuth, _directions, _absolute_directions;
        mutable bool _has_length, _has_azimuth, _has_directions;

        base_multi_rich_line()
                : _has_length{false}, _has_azimuth{false}, _has_directions{false} {}

        base_multi_rich_line(length_type length, angle_type azimuth, angle_type directions,
                             angle_type absolute_directions)
                : _length{length}, _has_length{true}, _has_azimuth{true}, _azimuth{azimuth}, _has_directions{true},
                  _directions{directions}, _absolute_directions{absolute_directions} {}

        template<typename RichLineT>
        void _transfer(const base_multi_rich_line<RichLineT> &other) {
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

    template<typename RichLine>
    class lazy_base_multi_rich_line : public base_multi_rich_line<RichLine> {

    public:
        using rich_line_type = RichLine;

        ~lazy_base_multi_rich_line() = default;

        lazy_base_multi_rich_line(const lazy_base_multi_rich_line &other)
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line(const lazy_base_multi_rich_line<RichLineT> &other)
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line(const base_multi_rich_line<RichLineT> &other)
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        lazy_base_multi_rich_line(lazy_base_multi_rich_line &&other) noexcept
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line(lazy_base_multi_rich_line<RichLineT> &&other) noexcept
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line(base_multi_rich_line<RichLineT> &&other) noexcept
                : base_multi_rich_line<rich_line_type>{} {
            _transfer(other);
        }

        lazy_base_multi_rich_line &operator=(const lazy_base_multi_rich_line &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line &operator=(const lazy_base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line &operator=(const base_multi_rich_line<RichLineT> &other) {
            _transfer(other);
            return *this;
        }

        lazy_base_multi_rich_line &operator=(lazy_base_multi_rich_line &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line &operator=(lazy_base_multi_rich_line<RichLineT> &&other) noexcept {
            _transfer(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_base_multi_rich_line &operator=(base_multi_rich_line<RichLineT> &&other) noexcept {
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

        lazy_base_multi_rich_line()
                : base_multi_rich_line<rich_line_type>{}, _computed_length{false}, _computed_azimuth{false},
                  _computed_directions{false} {}

        template<typename RichSegmentT>
        void _transfer(const lazy_base_multi_rich_line<RichSegmentT> &other) {
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
        void _transfer(const base_multi_rich_line<RichSegmentT> &other) {
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

    template<typename RichLine>
    class eager_multi_rich_line;

    template<typename RichLine>
    class lazy_multi_rich_line;

    template<typename RichLine>
    class multi_rich_line {

    public:
        using rich_line_type = RichLine;
        using segment_type = typename rich_line_traits<rich_line_type>::segment_type;
        using rich_segment_type = typename rich_line_traits<rich_line_type>::rich_segment_type;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using multi_line_type = typename rich_line_traits<rich_line_type>::multi_line_type;
        using point_type = typename rich_line_traits<rich_line_type>::point_type;
        using distance_type = typename rich_line_traits<rich_line_type>::distance_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        template<typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        multi_rich_line()
                : _rich_lines{} {}

        explicit multi_rich_line(const multi_line_type &multi_line)
                : multi_rich_line{multi_line2rich_lines<multi_rich_line, std::vector>(multi_line)} {}

        explicit multi_rich_line(const std::vector<line_type> &lines)
                : multi_rich_line{lines2multi_line<multi_rich_line>(lines)} {}

        explicit multi_rich_line(rich_line_type rich_line)
                : _rich_lines{std::move(rich_line)} {}

        explicit multi_rich_line(std::vector<rich_line_type> rich_lines)
                : _rich_lines{std::move(rich_lines)} {}

        template<typename RichLineT>
        explicit multi_rich_line(RichLineT rich_line)
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_line))} {}

        template<typename RichLineT>
        explicit multi_rich_line(std::vector<RichLineT> rich_lines)
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_lines))} {}

        ~multi_rich_line() = default;

        multi_rich_line(const multi_rich_line &other)
                : _rich_lines{other.rich_lines()} {}

        template<typename RichLineT>
        multi_rich_line(const multi_rich_line<RichLineT> &other)
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(other.rich_lines())} {}

        template<typename RichLineT>
        multi_rich_line(const eager_multi_rich_line<RichLineT> &other)
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(other.rich_lines())} {}

        template<typename RichLineT>
        multi_rich_line(const lazy_multi_rich_line<RichLineT> &other)
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(other.rich_lines())} {}

        multi_rich_line(multi_rich_line &&other) noexcept
                : _rich_lines{std::move(const_cast<std::vector<rich_line_type> &>(other.rich_lines()))} {}

        template<typename RichLineT>
        multi_rich_line(multi_rich_line<RichLineT> &&other) noexcept
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(
                std::move(const_cast<std::vector<RichLineT> &>(other.rich_lines())))} {}

        template<typename RichLineT>
        multi_rich_line(eager_multi_rich_line<RichLineT> &&other) noexcept
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(
                std::move(const_cast<std::vector<RichLineT> &>(other.rich_lines())))} {}

        template<typename RichLineT>
        multi_rich_line(lazy_multi_rich_line<RichLineT> &&other) noexcept
                : _rich_lines{convert_rich_types<rich_line_type, RichLineT>(
                std::move(const_cast<std::vector<RichLineT> &>(other.rich_lines())))} {}

        multi_rich_line &operator=(const multi_rich_line &other) {
            _rich_lines = other.rich_lines();
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(const multi_rich_line<RichLineT> &other) {
            _rich_lines = convert_rich_types<rich_line_type, RichLineT>(other.rich_lines());
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(const eager_multi_rich_line<RichLineT> &other) {
            _rich_lines = convert_rich_types<rich_line_type, RichLineT>(other.rich_lines());
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(const lazy_multi_rich_line<RichLineT> &other) {
            _rich_lines = convert_rich_types<rich_line_type, RichLineT>(other.rich_lines());
            return *this;
        }

        multi_rich_line &operator=(multi_rich_line &&other) noexcept {
            _rich_lines = std::move(const_cast<std::vector<rich_line_type> &>(other.rich_lines()));
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(multi_rich_line<RichLineT> &&other) noexcept {
            _rich_lines = convert_rich_types<rich_line_type, RichLineT>(
                    std::move(const_cast<std::vector<RichLineT> &>(other.rich_lines())));
            return *this;
        }

        template<typename RichLineT>
        multi_rich_line &operator=(eager_multi_rich_line<RichLineT> &&other) noexcept {
            _rich_lines = convert_rich_types<rich_line_type, RichLineT>(
                    std::move(const_cast<std::vector<RichLineT> &>(other.rich_lines())));
            return *this;
        }

        [[nodiscard]] bool has_length() const {
            if (not this->_rich_lines.empty()) {
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_length()) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] length_type length() const {
            if (not this->_rich_lines.empty()) {
                length_type length = default_float_type<length_type>::v0;
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_length()) {
                        length += rich_line.length();
                    }
                }
                return length;
            }

            throw std::runtime_error("multi_line has no length");
        }

        [[nodiscard]] bool has_azimuth() const {
            if (not this->_rich_lines.empty()) {
                if (this->_rich_lines.size() == 1) {
                    const auto &rich_line = this->_rich_lines.front();
                    if (rich_line.has_azimuth()) {
                        return true;
                    }
                } else {
                    const auto &first = this->_rich_lines.front().rich_segments().front().first();
                    const auto &second = this->_rich_lines.back().rich_segments().back().second();
                    if (not geometry::equals_points(first, second)) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] angle_type azimuth() const {
            if (not this->_rich_lines.empty()) {
                if (this->_rich_lines.size() == 1) {
                    const auto &rich_line = this->_rich_lines.front();
                    if (rich_line.has_azimuth()) {
                        return rich_line.azimuth();
                    }
                } else {
                    const auto &first = this->_rich_lines.front().rich_segments().front().first();
                    const auto &second = this->_rich_lines.back().rich_segments().back().second();
                    if (not geometry::equals_points(first, second)) {
                        return geometry::azimuth_deg(first, second);
                    }
                }
            }

            throw std::runtime_error("multi_line has no azimuth");
        }

        [[nodiscard]] bool has_directions() const {
            if (not this->_rich_lines.empty()) {
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_directions()) {
                        return true;
                    }
                }
            }

            return false;
        }

        [[nodiscard]] angle_type directions() const {
            if (not this->_rich_lines.empty()) {
                angle_type directions = default_float_type<angle_type>::v0;
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_directions()) {
                        directions += rich_line.directions();
                    }
                }
                return directions;
            }

            throw std::runtime_error("multi_line has no directions");
        }

        [[nodiscard]] angle_type absolute_directions() const {
            if (not this->_rich_lines.empty()) {
                angle_type absolute_directions = default_float_type<angle_type>::v0;
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_directions()) {
                        absolute_directions += rich_line.absolute_directions();
                    }
                }
                return absolute_directions;
            }

            throw std::runtime_error("multi_line has no absolute directions");
        }

        [[nodiscard]] const std::vector<rich_line_type> &rich_lines() const {
            return _rich_lines;
        }

        [[nodiscard]] multi_line_type multi_line() const {
            return rich_lines2multi_line<multi_rich_line>(_rich_lines);
        }

        [[nodiscard]] std::size_t sizes() const {
            std::size_t size = 0;
            for (const rich_line_type &rich_line: _rich_lines) {
                size += rich_line.size();
            }
            return size;
        }

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            for (auto &rich_line: _rich_lines) {
                rich_line.simplify(retain_reversals, tolerance, reverse_tolerance);
            }
        }

        template<typename Network, typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            for (auto &rich_line: _rich_lines) {
                rich_line.median_merge(tolerance, adaptive, network);
            }
        }

        template<typename MultiRichLine>
        [[nodiscard]] static MultiRichLine
        merge(const std::vector<std::reference_wrapper<typename MultiRichLine::rich_line_type>> &rich_lines) {
            std::vector<rich_line_type> new_rich_lines;

            auto it = rich_lines.cbegin();
            auto start_it = it;
            while (it != rich_lines.cend()) {
                if (it != rich_lines.cbegin()) {
                    const rich_line_type &prev_rich_line = *std::prev(it);
                    const rich_line_type &next_rich_line = *it;
                    if (not next_rich_line.rich_segments().empty()) {
                        if (not prev_rich_line.rich_segments().empty()) {
                            const auto &prev_point = prev_rich_line.back();
                            const auto &next_point = next_rich_line.front();
                            if (not geometry::equals_points(prev_point, next_point)) {
                                new_rich_lines.emplace_back(
                                        rich_line_type::template merge<rich_line_type>({start_it, it}));
                                start_it = it;
                            }
                        }
                    }
                }
                it++;
            }

            new_rich_lines.emplace_back(rich_line_type::template merge<rich_line_type>({start_it, it}));

            return MultiRichLine{std::move(new_rich_lines)};
        }

        [[nodiscard]] std::string wkt() const {
            if (_rich_lines.empty()) {
                return geometry::to_wkt(line_type{});
            } else if (_rich_lines.size() <= 1) {
                return geometry::to_wkt(_rich_lines.front().line());
            } else {
                return geometry::to_wkt(multi_line());
            }
        }

        [[nodiscard]] std::string str() const {
            std::string str = "MultiRichLine(";
            str.append(wkt());
            str.append(")");
            return str;
        }

        friend bool operator==(const multi_rich_line &left, const multi_rich_line &right) {
            // if size of lines is not equal, multi_lines cannot be equal
            if (left._rich_lines.size() != right._rich_lines.size()) {
                return false;
            }

            // if any lines is not equal, multi_lines cannot be equal
            for (std::size_t i = 0; i < left._rich_lines.size(); ++i) {
                if (left._rich_lines[i] != right._rich_lines[i]) {
                    return false;
                }
            }

            // if all lines are equal, multi_lines are equal
            return true;
        }

        friend bool operator!=(const multi_rich_line &left, const multi_rich_line &right) {
            return not(left == right);
        }

        friend std::ostream &operator<<(std::ostream &out, const multi_rich_line &multi_rich_line) {
            return out << multi_rich_line.str();
        }

    protected:
        std::vector<rich_line_type> _rich_lines;

    };

    template<typename RichLine>
    class eager_multi_rich_line : public multi_rich_line<RichLine>, public base_multi_rich_line<RichLine> {

    public:
        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using multi_line_type = typename rich_line_traits<rich_line_type>::multi_line_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        template<typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        eager_multi_rich_line()
                : multi_rich_line<rich_line_type>{}, base_multi_rich_line<rich_line_type>{} {}

        explicit eager_multi_rich_line(const multi_line_type &multi_line)
                : multi_rich_line<rich_line_type>{multi_line}, base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        explicit eager_multi_rich_line(const std::vector<line_type> &lines)
                : multi_rich_line<rich_line_type>{lines}, base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        explicit eager_multi_rich_line(rich_line_type rich_line)
                : multi_rich_line<rich_line_type>{std::move(rich_line)}, base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        explicit eager_multi_rich_line(std::vector<rich_line_type> rich_lines)
                : multi_rich_line<rich_line_type>{std::move(rich_lines)}, base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        template<typename RichLineT>
        explicit eager_multi_rich_line(RichLineT rich_line)
                : multi_rich_line<rich_line_type>{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_line))},
                  base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        template<typename RichLineT>
        explicit eager_multi_rich_line(std::vector<RichLineT> rich_lines)
                : multi_rich_line<rich_line_type>{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_lines))},
                  base_multi_rich_line<rich_line_type>{} {
            _compute();
        }

        ~eager_multi_rich_line() = default;

        eager_multi_rich_line(const eager_multi_rich_line &other)
                : multi_rich_line<rich_line_type>{other}, base_multi_rich_line<rich_line_type>{other} {}

        template<typename RichLineT>
        eager_multi_rich_line(const eager_multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other}, base_multi_rich_line<rich_line_type>{other} {}

        template<typename RichLineT>
        eager_multi_rich_line(const multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other} {
            _compute();
        }

        template<typename RichLineT>
        eager_multi_rich_line(const lazy_multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other} {
            _transfer(other);
        }

        eager_multi_rich_line(eager_multi_rich_line &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)},
                  base_multi_rich_line<rich_line_type>{std::move(other)} {}

        template<typename RichLineT>
        eager_multi_rich_line(eager_multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)},
                  base_multi_rich_line<rich_line_type>{std::move(other)} {}

        template<typename RichLineT>
        eager_multi_rich_line(multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)} {
            _compute();
        }

        template<typename RichLineT>
        eager_multi_rich_line(lazy_multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)} {
            _transfer(other);
        }

        eager_multi_rich_line &operator=(const eager_multi_rich_line &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            base_multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(const eager_multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            base_multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(const multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            _compute();
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(const lazy_multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            _transfer(other);
            return *this;
        }

        eager_multi_rich_line &operator=(eager_multi_rich_line &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            base_multi_rich_line<rich_line_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(eager_multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            base_multi_rich_line<rich_line_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            _compute();
            return *this;
        }

        template<typename RichLineT>
        eager_multi_rich_line &operator=(lazy_multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
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

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            multi_rich_line<rich_line_type>::simplify(retain_reversals, tolerance, reverse_tolerance);
            _compute();
        }

        template<typename Network, typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            multi_rich_line<rich_line_type>::median_merge(tolerance, adaptive, network);
            _compute();
        }

        [[nodiscard]] std::string str() const {
            std::string str = "EagerMultiRichLine(";
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

    private:
        void _compute_length() const {
            this->_has_length = false;
            if (not this->_rich_lines.empty()) {
                this->_length = default_float_type<length_type>::v0;
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_length()) {
                        this->_length += rich_line.length();
                        this->_has_length = true;
                    }
                }
            }
        }

        void _compute_azimuth() const {
            this->_has_azimuth = false;
            if (not this->_rich_lines.empty()) {
                if (this->_rich_lines.size() == 1) {
                    const auto &rich_line = this->_rich_lines.front();
                    if (rich_line.has_azimuth()) {
                        this->_azimuth = rich_line.azimuth();
                        this->_has_azimuth = true;
                    }
                } else {
                    const auto &first = this->_rich_lines.front().rich_segments().front().first();
                    const auto &second = this->_rich_lines.back().rich_segments().back().second();
                    if (not geometry::equals_points(first, second)) {
                        this->_azimuth = geometry::azimuth_deg(first, second);
                        this->_has_azimuth = true;
                    }
                }
            }
        }

        void _compute_directions() const {
            this->_has_directions = false;
            if (not this->_rich_lines.empty()) {
                this->_directions = default_float_type<angle_type>::v0;
                this->_absolute_directions = default_float_type<angle_type>::v0;
                for (const auto &rich_line: this->_rich_lines) {
                    if (rich_line.has_directions()) {
                        this->_directions += rich_line.directions();
                        this->_absolute_directions += rich_line.absolute_directions();
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

        template<typename RichLineT>
        void _transfer(const lazy_base_multi_rich_line<RichLineT> &other) {
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

    template<typename RichLine>
    class lazy_multi_rich_line : public multi_rich_line<RichLine>, public lazy_base_multi_rich_line<RichLine> {

    public:
        using rich_line_type = RichLine;
        using line_type = typename rich_line_traits<rich_line_type>::line_type;
        using multi_line_type = typename rich_line_traits<rich_line_type>::multi_line_type;
        using length_type = typename rich_line_traits<rich_line_type>::length_type;
        using angle_type = typename rich_line_traits<rich_line_type>::angle_type;

        template<typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        lazy_multi_rich_line()
                : multi_rich_line<rich_line_type>{}, lazy_base_multi_rich_line<rich_line_type>{} {}

        explicit lazy_multi_rich_line(const multi_line_type &multi_line)
                : multi_rich_line<rich_line_type>{multi_line}, lazy_base_multi_rich_line<rich_line_type>{} {}

        explicit lazy_multi_rich_line(const std::vector<line_type> &lines)
                : multi_rich_line<rich_line_type>{lines}, lazy_base_multi_rich_line<rich_line_type>{} {}

        explicit lazy_multi_rich_line(rich_line_type rich_line)
                : multi_rich_line<rich_line_type>{std::move(rich_line)}, lazy_base_multi_rich_line<rich_line_type>{} {}

        explicit lazy_multi_rich_line(std::vector<rich_line_type> rich_lines)
                : multi_rich_line<rich_line_type>{std::move(rich_lines)}, lazy_base_multi_rich_line<rich_line_type>{} {}

        template<typename RichLineT>
        explicit lazy_multi_rich_line(RichLineT rich_line)
                : multi_rich_line<rich_line_type>{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_line))},
                  lazy_base_multi_rich_line<rich_line_type>{} {}

        template<typename RichLineT>
        explicit lazy_multi_rich_line(std::vector<RichLineT> rich_lines)
                : multi_rich_line<rich_line_type>{convert_rich_types<rich_line_type, RichLineT>(std::move(rich_lines))},
                  lazy_base_multi_rich_line<rich_line_type>{} {}

        ~lazy_multi_rich_line() = default;

        lazy_multi_rich_line(const lazy_multi_rich_line &other)
                : multi_rich_line<rich_line_type>{other}, lazy_base_multi_rich_line<rich_line_type>{other} {}

        template<typename RichLineT>
        lazy_multi_rich_line(const lazy_multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other}, lazy_base_multi_rich_line<rich_line_type>{other} {}

        template<typename RichLineT>
        lazy_multi_rich_line(const multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other}, lazy_base_multi_rich_line<rich_line_type>{} {}

        template<typename RichLineT>
        lazy_multi_rich_line(const eager_multi_rich_line<RichLineT> &other)
                : multi_rich_line<rich_line_type>{other}, lazy_base_multi_rich_line<rich_line_type>{other} {}

        lazy_multi_rich_line(lazy_multi_rich_line &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)},
                  lazy_base_multi_rich_line<rich_line_type>{std::move(other)} {}

        template<typename RichLineT>
        lazy_multi_rich_line(lazy_multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)},
                  lazy_base_multi_rich_line<rich_line_type>{std::move(other)} {}

        template<typename RichLineT>
        lazy_multi_rich_line(multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)}, lazy_base_multi_rich_line<rich_line_type>{} {}

        template<typename RichLineT>
        lazy_multi_rich_line(eager_multi_rich_line<RichLineT> &&other) noexcept
                : multi_rich_line<rich_line_type>{std::move(other)},
                  lazy_base_multi_rich_line<rich_line_type>{std::move(other)} {}

        lazy_multi_rich_line &operator=(const lazy_multi_rich_line &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            lazy_base_multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(const lazy_multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            lazy_base_multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(const multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(const eager_multi_rich_line<RichLineT> &other) {
            multi_rich_line<rich_line_type>::operator=(other);
            lazy_base_multi_rich_line<rich_line_type>::operator=(other);
            return *this;
        }

        lazy_multi_rich_line &operator=(lazy_multi_rich_line &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            lazy_base_multi_rich_line<rich_line_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(lazy_multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            lazy_base_multi_rich_line<rich_line_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            return *this;
        }

        template<typename RichLineT>
        lazy_multi_rich_line &operator=(eager_multi_rich_line<RichLineT> &&other) noexcept {
            multi_rich_line<rich_line_type>::operator=(std::move(other));
            lazy_base_multi_rich_line<rich_line_type>::operator=(std::move(other));
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

        void simplify(bool retain_reversals = true, double tolerance = 1e-3, double reverse_tolerance = 1e-3) {
            multi_rich_line<rich_line_type>::simplify(retain_reversals, tolerance, reverse_tolerance);
            this->_computed_length.store(false, std::memory_order_release);
            this->_computed_azimuth.store(false, std::memory_order_release);
            this->_computed_directions.store(false, std::memory_order_release);
        }

        template<typename Network, typename RichLineT = rich_line_type,
                typename = std::enable_if_t<std::is_default_constructible_v<RichLineT>>>
        void median_merge(double tolerance = 1e-3, bool adaptive = true, const Network *network = nullptr) {
            multi_rich_line<rich_line_type>::median_merge(tolerance, adaptive, network);
            this->_computed_length.store(false, std::memory_order_release);
            this->_computed_azimuth.store(false, std::memory_order_release);
            this->_computed_directions.store(false, std::memory_order_release);
        }

        [[nodiscard]] std::string str() const {
            std::string str = "LazyMultiRichLine(";
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

    private:
        void _compute_length() const {
            if (not this->_computed_length.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(this->_mutex);
                if (not this->_computed_length.load(std::memory_order_relaxed)) {
                    this->_has_length = false;
                    if (not this->_rich_lines.empty()) {
                        this->_length = default_float_type<length_type>::v0;
                        for (const auto &rich_line: this->_rich_lines) {
                            if (rich_line.has_length()) {
                                this->_length += rich_line.length();
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
                    if (not this->_rich_lines.empty()) {
                        if (this->_rich_lines.size() == 1) {
                            const auto &rich_line = this->_rich_lines.front();
                            if (rich_line.has_azimuth()) {
                                this->_azimuth = rich_line.azimuth();
                                this->_has_azimuth = true;
                            }
                        } else {
                            const auto &first = this->_rich_lines.front().rich_segments().front().first();
                            const auto &second = this->_rich_lines.back().rich_segments().back().second();
                            if (not geometry::equals_points(first, second)) {
                                this->_azimuth = geometry::azimuth_deg(first, second);
                                this->_has_azimuth = true;
                            }
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
                    if (not this->_rich_lines.empty()) {
                        this->_directions = default_float_type<angle_type>::v0;
                        this->_absolute_directions = default_float_type<angle_type>::v0;
                        for (const auto &rich_line: this->_rich_lines) {
                            if (rich_line.has_directions()) {
                                this->_directions += rich_line.directions();
                                this->_absolute_directions += rich_line.absolute_directions();
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
    using multi_rich_line_type = multi_rich_line<rich_line_type<Point>>;

    template<typename Point>
    using multi_rich_referring_line_type = multi_rich_line<rich_referring_line_type<Point>>;

    template<typename Point>
    using multi_rich_pointing_line_type = multi_rich_line<rich_pointing_line_type<Point>>;

    template<typename Point>
    using eager_multi_rich_line_type = eager_multi_rich_line<eager_rich_line_type<Point>>;

    template<typename Point>
    using eager_multi_rich_referring_line_type = eager_multi_rich_line<eager_rich_referring_line_type<Point>>;

    template<typename Point>
    using eager_multi_rich_pointing_line_type = eager_multi_rich_line<eager_rich_pointing_line_type<Point>>;

    template<typename Point>
    using lazy_multi_rich_line_type = lazy_multi_rich_line<lazy_rich_line_type<Point>>;

    template<typename Point>
    using lazy_multi_rich_referring_line_type = lazy_multi_rich_line<lazy_rich_referring_line_type<Point>>;

    template<typename Point>
    using lazy_multi_rich_pointing_line_type = lazy_multi_rich_line<lazy_rich_pointing_line_type<Point>>;

}

#endif //MAP_MATCHING_2_MULTI_RICH_LINE_HPP
