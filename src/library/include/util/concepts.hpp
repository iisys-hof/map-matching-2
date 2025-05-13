// Copyright (C) 2023-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_UTIL_CONCEPTS_HPP
#define MAP_MATCHING_2_UTIL_CONCEPTS_HPP

#include <concepts>
#include <variant>
#include <iterator>

namespace map_matching_2::util {

    template<typename>
    inline constexpr bool dependent_false_v = false;

    template<typename Container>
    using is_forward_access = std::is_base_of<std::forward_iterator_tag,
        typename std::iterator_traits<typename Container::iterator>::iterator_category>;

    template<typename Container>
    static inline constexpr bool is_forward_access_v = is_forward_access<Container>::value;

    template<typename Container>
    using is_bidirectional_access = std::is_base_of<std::bidirectional_iterator_tag,
        typename std::iterator_traits<typename Container::iterator>::iterator_category>;

    template<typename Container>
    static inline constexpr bool is_bidirectional_access_v = is_bidirectional_access<Container>::value;

    template<typename Container>
    using is_random_access = std::is_base_of<std::random_access_iterator_tag,
        typename std::iterator_traits<typename Container::iterator>::iterator_category>;

    template<typename Container>
    static inline constexpr bool is_random_access_v = is_random_access<Container>::value;

    template<typename Container, typename Iterator>
    concept is_iterator_of_container = std::same_as<Iterator, typename Container::iterator> ||
            std::same_as<Iterator, typename Container::const_iterator>;

    template<typename Container>
    concept has_reserve = requires(Container &container, typename Container::size_type size) {
        { container.reserve(size) } -> std::same_as<void>;
    };

    template<typename Container>
    concept has_capacity = requires(Container &container) {
        { container.capacity() } -> std::same_as<std::size_t>;
    };

    template<typename T, typename... U>
    concept is_any_of = (std::same_as<T, U> || ...);

    template<typename T, typename... U>
    inline constexpr bool is_any_of_v = is_any_of<T, U...>;

    template<template<typename...> typename Wrapper, typename VariantType, bool IsConst>
    struct variant_wrapper;

    template<template<typename...> typename Wrapper, typename... Types>
    struct variant_wrapper<Wrapper, std::variant<Types...>, false> {
        using type = std::variant<Wrapper<Types>...>;
    };

    template<template<typename...> typename Wrapper, typename... Types>
    struct variant_wrapper<Wrapper, std::variant<Types...>, true> {
        using type = std::variant<Wrapper<const Types>...>;
    };

    template<template<typename...> typename Wrapper, typename VariantType, bool IsConst>
    using variant_wrapper_t = typename variant_wrapper<Wrapper, VariantType, IsConst>::type;

    template<typename V, template<typename> typename C>
    struct variant_satisfies : std::false_type {};

    template<template<typename> typename C, typename... U>
    struct variant_satisfies<std::variant<U...>, C>
            : std::bool_constant<(C<U>::value && ...)> {};

    template<typename V, template<typename> typename C>
    concept variant_satisfies_t = variant_satisfies<V, C>::value;

    template<typename V, typename T>
    struct variant_alternative : std::false_type {};

    template<typename T, typename... U>
    struct variant_alternative<std::variant<U...>, T>
            : std::bool_constant<is_any_of<T, U...>> {};

    template<typename V, typename T>
    concept variant_alternative_t = variant_alternative<V, T>::value;

    template<typename Type, typename Other, typename Allocator>
    concept has_allocator_conversion_constructor = requires(Other &other, Allocator &allocator) {
        Type(other, allocator);
    };

}

#endif //MAP_MATCHING_2_UTIL_CONCEPTS_HPP
