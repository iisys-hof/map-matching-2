// Copyright (C) 2026 Adrian Wöltche
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

#include <optional>

#include "util/math.hpp"

BOOST_AUTO_TEST_SUITE(algorithm_tests)

    BOOST_AUTO_TEST_CASE(gaussian_weight_test) {
        constexpr std::size_t size = 11;
        constexpr std::size_t center = 5;
        constexpr double min_weight = 2.0;
        constexpr double max_weight = 5.0;

        std::vector<double> weights;
        weights.reserve(size);
        std::optional<double> prev_weight;
        for (std::size_t i = 0; i < size; ++i) {
            auto weight = map_matching_2::util::gaussian_weight(i, center, size, min_weight, max_weight, 2.0);
            if (prev_weight.has_value()) {
                if (i < center) {
                    BOOST_CHECK_LT(prev_weight.value(), weight);
                } else if (i == center) {
                    BOOST_CHECK_CLOSE(weight, max_weight, 1e-6);
                } else {
                    BOOST_CHECK_GT(prev_weight.value(), weight);
                }
            }
            prev_weight = weight;
        }
    }

    BOOST_AUTO_TEST_CASE(interpolation_test) {
        const auto i_1 = map_matching_2::util::interpolate(1, 9, 5);
        const std::vector i_1_e{1, 3, 5, 7, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_1), std::cend(i_1), std::cbegin(i_1_e), std::cend(i_1_e));

        const auto i_2 = map_matching_2::util::interpolate(1.0, 9.0, 5);
        const std::vector i_2_e{1.0, 3.0, 5.0, 7.0, 9.0};
        BOOST_CHECK_EQUAL(i_2.size(), i_2_e.size());
        for (std::size_t i = 0; i < i_2.size(); ++i) {
            BOOST_CHECK_CLOSE(i_2[i], i_2_e[i], 1e-6);
        }

        const auto i_3 = map_matching_2::util::interpolate(1, 9, 3);
        const std::vector i_3_e{1, 5, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_3), std::cend(i_3), std::cbegin(i_3_e), std::cend(i_3_e));

        const auto i_4 = map_matching_2::util::interpolate(1, 9, 9);
        const std::vector i_4_e{1, 2, 3, 4, 5, 6, 7, 8, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_4), std::cend(i_4), std::cbegin(i_4_e), std::cend(i_4_e));

        const auto i_5 = map_matching_2::util::interpolate(1, 9, 0);
        BOOST_CHECK(i_5.empty());

        const auto i_6 = map_matching_2::util::interpolate(1, 9, 1);
        const std::vector i_6_e{1};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_6), std::cend(i_6), std::cbegin(i_6_e), std::cend(i_6_e));

        const auto i_7 = map_matching_2::util::interpolate(1, 9, 2);
        const std::vector i_7_e{1, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_7), std::cend(i_7), std::cbegin(i_7_e), std::cend(i_7_e));
    }

    BOOST_AUTO_TEST_CASE(interpolation_by_distance_test) {
        std::vector<double> d_1{1, 1, 1, 1};
        const auto i_1 = map_matching_2::util::interpolate_by_distance(1, 9, d_1);
        const std::vector i_1_e{1, 3, 5, 7, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_1), std::cend(i_1), std::cbegin(i_1_e), std::cend(i_1_e));

        const auto i_2 = map_matching_2::util::interpolate_by_distance(1.0, 9.0, d_1);
        const std::vector i_2_e{1.0, 3.0, 5.0, 7.0, 9.0};
        BOOST_CHECK_EQUAL(i_2.size(), i_2_e.size());
        for (std::size_t i = 0; i < i_2.size(); ++i) {
            BOOST_CHECK_CLOSE(i_2[i], i_2_e[i], 1e-6);
        }

        std::vector<double> d_2{0, 0, 0, 0};
        const auto i_3 = map_matching_2::util::interpolate_by_distance(1, 9, d_2);
        const std::vector i_3_e{1, 1, 1, 1, 1};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_3), std::cend(i_3), std::cbegin(i_3_e), std::cend(i_3_e));

        std::vector<double> d_3{0, 0, 1, 0};
        const auto i_4 = map_matching_2::util::interpolate_by_distance(1, 9, d_3);
        const std::vector i_4_e{1, 1, 1, 9, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_4), std::cend(i_4), std::cbegin(i_4_e), std::cend(i_4_e));

        std::vector<double> d_4{1, 0, 1, 0};
        const auto i_5 = map_matching_2::util::interpolate_by_distance(1, 9, d_4);
        const std::vector i_5_e{1, 5, 5, 9, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_5), std::cend(i_5), std::cbegin(i_5_e), std::cend(i_5_e));

        std::vector<double> d_5{0, 1, 0, 1};
        const auto i_6 = map_matching_2::util::interpolate_by_distance(1, 9, d_5);
        const std::vector i_6_e{1, 1, 5, 5, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_6), std::cend(i_6), std::cbegin(i_6_e), std::cend(i_6_e));

        std::vector<double> d_6{1, 2, 1};
        const auto i_7 = map_matching_2::util::interpolate_by_distance(1, 9, d_6);
        const std::vector i_7_e{1, 3, 7, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_7), std::cend(i_7), std::cbegin(i_7_e), std::cend(i_7_e));

        std::vector<double> d_7;
        const auto i_8 = map_matching_2::util::interpolate_by_distance(1, 9, d_7);
        BOOST_CHECK(i_8.empty());

        std::vector<double> d_8{1};
        const auto i_9 = map_matching_2::util::interpolate_by_distance(1, 9, d_8);
        const std::vector i_9_e{1, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_9), std::cend(i_9), std::cbegin(i_9_e), std::cend(i_9_e));

        std::vector<double> d_9{0};
        const auto i_10 = map_matching_2::util::interpolate_by_distance(1, 9, d_9);
        const std::vector i_10_e{1, 1};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_10), std::cend(i_10), std::cbegin(i_10_e), std::cend(i_10_e));

        std::vector<double> d_10{3};
        const auto i_11 = map_matching_2::util::interpolate_by_distance(1, 9, d_10);
        const std::vector i_11_e{1, 9};
        BOOST_CHECK_EQUAL_COLLECTIONS(std::cbegin(i_11), std::cend(i_11), std::cbegin(i_11_e), std::cend(i_11_e));
    }

BOOST_AUTO_TEST_SUITE_END()
