// Copyright (C) 2023-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_MODELS_HPP
#define MAP_MATCHING_2_GEOMETRY_MODELS_HPP

#include <vector>

#include <boost/concept/assert.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/geometry/geometries/linestring.hpp>

namespace map_matching_2::geometry {

    template<typename Point,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    class linestring : public Container<Point, Allocator<Point>> {

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Point<Point>));

    public:
        using point_type = Point;
        using base_type = Container<point_type, Allocator<point_type>>;

        using allocator_type = Allocator<void>;

        using size_type = typename base_type::size_type;

        constexpr linestring() requires
            (std::default_initializable<allocator_type>)
            : base_type{} {}

        constexpr explicit linestring(const allocator_type &alloc) noexcept
            : base_type(alloc) {}

        constexpr linestring(size_type count, const point_type &value) requires
            (std::default_initializable<allocator_type>)
            : base_type{count, value} {}

        constexpr linestring(size_type count, const point_type &value, const allocator_type &alloc)
            : base_type{count, value, alloc} {}

        constexpr explicit linestring(size_type count) requires
            (std::default_initializable<allocator_type>)
            : base_type(count) {}

        constexpr linestring(size_type count, const allocator_type &alloc)
            : base_type{count, alloc} {}

        template<typename InputIterator>
        constexpr linestring(InputIterator first, InputIterator last) requires
            (std::default_initializable<allocator_type>)
            : base_type{std::move(first), std::move(last)} {}

        template<typename InputIterator>
        constexpr linestring(InputIterator first, InputIterator last, const allocator_type &alloc)
            : base_type{std::move(first), std::move(last), alloc} {}

        constexpr linestring(const linestring &other)
            : base_type{other} {}

        constexpr linestring(const linestring &other, const allocator_type &alloc)
            : base_type{other, alloc} {}

        template<typename PointT = Point,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr linestring(const linestring<PointT, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<allocator_type>)
            : base_type{} {
            std::copy(std::cbegin(other), std::cend(other), std::begin(*this));
        }

        template<typename PointT = Point,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr linestring(const linestring<PointT, ContainerT, AllocatorT> &other,
                const allocator_type &alloc)
            : base_type(alloc) {
            std::copy(std::cbegin(other), std::cend(other), std::begin(*this));
        }

        constexpr linestring(linestring &&other) noexcept
            : base_type{std::move(other)} {}

        constexpr linestring(linestring &&other, const allocator_type &alloc) noexcept
            : base_type{std::move(other), alloc} {}

        constexpr linestring(std::initializer_list<point_type> init) requires
            (std::default_initializable<allocator_type>)
            : base_type{std::move(init)} {}

        constexpr linestring(std::initializer_list<point_type> init, const allocator_type &alloc)
            : base_type{std::move(init), alloc} {}

        constexpr ~linestring() = default;

        constexpr linestring &operator=(const linestring &other) {
            if (this != &other) {
                base_type::operator=(other);
            }
            return *this;
        }

        constexpr linestring &operator=(linestring &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr linestring &operator=(std::initializer_list<point_type> ilist) {
            base_type::operator=(std::move(ilist));
            return *this;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<base_type>(*this);
        }

    };

}

namespace boost::geometry::traits {

    template<typename Point,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    struct tag<map_matching_2::geometry::linestring<Point, Container, Allocator>> {
        typedef linestring_tag type;
    };

}

namespace map_matching_2::geometry {

    template<typename Line>
    concept is_line = std::is_same_v<typename boost::geometry::tag<Line>::type, boost::geometry::linestring_tag>;

}

namespace map_matching_2::geometry {

    template<typename Line,
        template<typename, typename> typename Container = std::vector,
        template<typename> typename Allocator = std::allocator>
    class multi_linestring : public Container<Line, Allocator<Line>> {

        BOOST_CONCEPT_ASSERT((boost::geometry::concepts::Linestring<Line>));

    public:
        using line_type = Line;
        using base_type = Container<line_type, Allocator<line_type>>;

        using allocator_type = Allocator<void>;

        using size_type = typename base_type::size_type;

        constexpr multi_linestring() requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : base_type{} {}

        constexpr explicit multi_linestring(const allocator_type &alloc) noexcept requires
            (std::default_initializable<line_type>)
            : base_type(alloc) {}

        constexpr multi_linestring(size_type count, const line_type &value) requires
            (std::default_initializable<allocator_type>)
            : base_type{count, value} {}

        constexpr multi_linestring(size_type count, const line_type &value, const allocator_type &alloc)
            : base_type{count, value, alloc} {}

        constexpr explicit multi_linestring(size_type count) requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : base_type(count) {}

        constexpr multi_linestring(size_type count, const allocator_type &alloc) requires
            (std::default_initializable<line_type>)
            : base_type{count, alloc} {}

        template<typename InputIterator>
        constexpr multi_linestring(InputIterator first, InputIterator last) requires
            (std::default_initializable<allocator_type>)
            : base_type{std::move(first), std::move(last)} {}

        template<typename InputIterator>
        constexpr multi_linestring(InputIterator first, InputIterator last, const allocator_type &alloc)
            : base_type{std::move(first), std::move(last), alloc} {}

        constexpr multi_linestring(const multi_linestring &other)
            : base_type{other} {}

        constexpr multi_linestring(const multi_linestring &other, const allocator_type &alloc)
            : base_type{other, alloc} {}

        template<typename LineT = Line,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr multi_linestring(const linestring<LineT, ContainerT, AllocatorT> &other) requires
            (std::default_initializable<line_type> and std::default_initializable<allocator_type>)
            : base_type{} {
            std::copy(std::cbegin(other), std::cend(other), std::begin(*this));
        }

        template<typename LineT = Line,
            template<typename, typename> typename ContainerT = Container,
            template<typename> typename AllocatorT = Allocator>
        constexpr multi_linestring(const linestring<LineT, ContainerT, AllocatorT> &other,
                const allocator_type &alloc) requires
            (std::default_initializable<line_type>)
            : base_type(alloc) {
            std::copy(std::cbegin(other), std::cend(other), std::begin(*this));
        }

        constexpr multi_linestring(multi_linestring &&other) noexcept
            : base_type{std::move(other)} {}

        constexpr multi_linestring(multi_linestring &&other, const allocator_type &alloc) noexcept
            : base_type{std::move(other), alloc} {}

        constexpr multi_linestring(std::initializer_list<line_type> init) requires
            (std::default_initializable<allocator_type>)
            : base_type{std::move(init)} {}

        constexpr multi_linestring(std::initializer_list<line_type> init, const allocator_type &alloc)
            : base_type{std::move(init), alloc} {}

        constexpr ~multi_linestring() = default;

        constexpr multi_linestring &operator=(const multi_linestring &other) {
            if (this != &other) {
                base_type::operator=(other);
            }
            return *this;
        }

        constexpr multi_linestring &operator=(multi_linestring &&other) noexcept {
            if (this != &other) {
                base_type::operator=(std::move(other));
            }
            return *this;
        }

        constexpr multi_linestring &operator=(std::initializer_list<line_type> ilist) {
            base_type::operator=(std::move(ilist));
            return *this;
        }

        friend class boost::serialization::access;

        template<typename Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<base_type>(*this);
        }

    };

}

namespace boost::geometry::traits {

    template<typename Line,
        template<typename, typename> typename Container,
        template<typename> typename Allocator>
    struct tag<map_matching_2::geometry::multi_linestring<Line, Container, Allocator>> {
        typedef multi_linestring_tag type;
    };

}

namespace map_matching_2::geometry {

    template<typename MultiLine>
    concept is_multi_line = std::is_same_v<typename boost::geometry::tag<MultiLine>::type,
        boost::geometry::multi_linestring_tag>;

}

namespace boost::serialization {

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::segment<Point> &segment, const unsigned int version) {
        ar & boost::serialization::make_nvp("segment", static_cast<std::pair<Point, Point> &>(segment));
    }

    template<typename Archive, typename Point>
    void serialize(Archive &ar, boost::geometry::model::linestring<Point> &line, const unsigned int version) {
        ar & boost::serialization::make_nvp("line", static_cast<std::vector<Point> &>(line));
    }

}

#endif //MAP_MATCHING_2_GEOMETRY_MODELS_HPP
