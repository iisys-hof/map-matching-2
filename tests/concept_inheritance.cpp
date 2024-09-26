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

#include <boost/test/unit_test.hpp>

struct A {
    using type = int;
};

struct B {
    using type = float;
};

template<typename T>
concept has_type = requires {
    typename T::type;
};

template<typename T>
concept is_int = has_type<T> && std::same_as<typename T::type, int>;

template<typename T>
concept is_float = has_type<T> && std::same_as<typename T::type, float>;

template<typename T>
struct base {};

template<typename T> requires is_int<T>
struct base<T> {

    base() {};

};

template<typename T> requires is_float<T>
struct base<T> {

    base(T::type data)
        : _data{data} {}

    T::type _data;

};

template<typename T> requires has_type<T>
struct concrete : public base<T> {

    using base_type = base<T>;

    concrete() requires is_int<T>
        : base_type{} {}

    concrete(T::type data) requires is_float<T>
        : base_type{data} {}

};

template<typename T> requires has_type<T>
struct sub : public concrete<T> {

    using base_type = concrete<T>;

    sub() requires is_int<T>
        : base_type{} {}

    sub(T::type data) requires is_float<T>
        : base_type{data} {}

};

BOOST_AUTO_TEST_SUITE(concept_inheritance)

    BOOST_AUTO_TEST_CASE(concept_inheritance_test) {

        auto a = sub<A>{};
        auto b = sub<B>{2.0};

        BOOST_CHECK_EQUAL(b._data, 2.0);

    }

BOOST_AUTO_TEST_SUITE_END()
