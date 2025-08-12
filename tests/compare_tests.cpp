// Copyright (C) 2021-2025 Adrian Wöltche
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

#include "geometry/algorithm/compare.hpp"
#include "geometry/rich_type/rich_line.hpp"
#include "geometry/rich_type/multi_rich_line.hpp"

#include "helper/geometry_helper.hpp"

using point_type = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
using line_type = map_matching_2::geometry::models<point_type>::line_type<>;
using rich_line_type = map_matching_2::geometry::rich_line<point_type>;
using multi_rich_line_type = map_matching_2::geometry::multi_rich_line<rich_line_type>;

BOOST_AUTO_TEST_SUITE(compare_tests)

    BOOST_AUTO_TEST_CASE(compare_test) {
        map_matching_2::geometry::line_comparator<multi_rich_line_type> line_comparator;

        // correct vertical
        auto line_1_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_1_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_1 = line_comparator.compare(line_1_1, line_1_2);
        BOOST_CHECK_EQUAL(result_1.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_1.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_1.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_1.correct, 5.0);
        BOOST_CHECK_EQUAL(result_1.correct_fraction, 1.0);

        // correct horizontal
        auto line_2_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0})
        };
        auto line_2_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0})
        };
        auto result_2 = line_comparator.compare(line_2_1, line_2_2);
        BOOST_CHECK_EQUAL(result_2.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_2.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_2.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_2.correct, 5.0);
        BOOST_CHECK_EQUAL(result_2.correct_fraction, 1.0);

        // correct shorted a
        auto line_3_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 3, 0, 5, 0})};
        auto line_3_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0})
        };
        auto result_3 = line_comparator.compare(line_3_1, line_3_2);
        BOOST_CHECK_EQUAL(result_3.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_3.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_3.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_3.correct, 5.0);
        BOOST_CHECK_EQUAL(result_3.correct_fraction, 1.0);

        // correct shorted b
        auto line_4_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0})
        };
        auto line_4_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 2, 0, 5, 0})};
        auto result_4 = line_comparator.compare(line_4_1, line_4_2);
        BOOST_CHECK_EQUAL(result_4.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_4.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_4.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_4.correct, 5.0);
        BOOST_CHECK_EQUAL(result_4.correct_fraction, 1.0);

        // correct shorted both
        auto line_5_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 2, 0, 5, 0})};
        auto line_5_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 3, 0, 5, 0})};
        auto result_5 = line_comparator.compare(line_5_1, line_5_2);
        BOOST_CHECK_EQUAL(result_5.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_5.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_5.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_5.correct, 5.0);
        BOOST_CHECK_EQUAL(result_5.correct_fraction, 1.0);

        // opposite direction
        auto line_6_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 3, 0, 5})};
        auto line_6_2 = rich_line_type{create_line<point_type, line_type>({0, 5, 0, 2, 0, 0})};
        auto result_6 = line_comparator.compare(line_6_1, line_6_2);
        BOOST_CHECK_CLOSE_FRACTION(result_6.error_added, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_6.error_missed, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_6.error_fraction, 10.0 / 5, 1e-6);
        BOOST_CHECK_EQUAL(result_6.correct, 0.0);
        BOOST_CHECK_EQUAL(result_6.correct_fraction, 0.0);

        // missing start
        auto line_7_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 2, 0, 5})};
        auto line_7_2 = rich_line_type{create_line<point_type, line_type>({0, 1, 0, 3, 0, 5})};
        auto result_7 = line_comparator.compare(line_7_1, line_7_2);
        BOOST_CHECK_EQUAL(result_7.error_added, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_7.error_missed, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_7.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_7.correct, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_7.correct_fraction, 4.0 / 5, 1e-6);

        // missing end
        auto line_8_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 2, 0, 5})};
        auto line_8_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 1, 0, 4})};
        auto result_8 = line_comparator.compare(line_8_1, line_8_2);
        BOOST_CHECK_EQUAL(result_8.error_added, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_8.error_missed, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_8.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_8.correct, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_8.correct_fraction, 4.0 / 5, 1e-6);

        // added start
        auto line_9_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 5})};
        auto line_9_2 = rich_line_type{create_line<point_type, line_type>({1, 0, 0, 0, 0, 1, 0, 2, 0, 5})};
        auto result_9 = line_comparator.compare(line_9_1, line_9_2);
        BOOST_CHECK_CLOSE_FRACTION(result_9.error_added, 1.0, 1e-6);
        BOOST_CHECK_EQUAL(result_9.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_9.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_9.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_9.correct_fraction, 5.0 / 6, 1e-6);

        // added end
        auto line_10_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 4, 0, 5})};
        auto line_10_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 4, 0, 5, 1, 5})};
        auto result_10 = line_comparator.compare(line_10_1, line_10_2);
        BOOST_CHECK_CLOSE_FRACTION(result_10.error_added, 1.0, 1e-6);
        BOOST_CHECK_EQUAL(result_10.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_10.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_10.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_10.correct_fraction, 5.0 / 6, 1e-6);

        // added missing
        auto line_11_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_11_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 1, 1, 1, 2, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_11 = line_comparator.compare(line_11_1, line_11_2);
        BOOST_CHECK_CLOSE_FRACTION(result_11.error_added, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_11.error_missed, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_11.error_fraction, 4.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_11.correct, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_11.correct_fraction, 4.0 / 7, 1e-6);

        // outside part
        auto line_12_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_12_2 = rich_line_type{
                create_line<point_type, line_type>({1, 0, 1, 1, 0, 1, 0, 2, 1, 2, 1, 3, 1, 4})
        };
        auto result_12 = line_comparator.compare(line_12_1, line_12_2);
        BOOST_CHECK_CLOSE_FRACTION(result_12.error_added, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_12.error_missed, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_12.error_fraction, 9.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_12.correct, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_12.correct_fraction, 1.0 / 6, 1e-6);

        // reverse within
        auto line_13_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_13_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 2, 0, 1, 0, 5})};
        auto result_13 = line_comparator.compare(line_13_1, line_13_2);
        BOOST_CHECK_CLOSE_FRACTION(result_13.error_added, 2.0, 1e-6);
        BOOST_CHECK_EQUAL(result_13.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_13.error_fraction, 2.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_13.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_13.correct_fraction, 5.0 / 7, 1e-6);

        // reverse within overlap
        auto line_14_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_14_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 2, 0, 4, 0, 2, 0, 1, 0, 5})
        };
        auto result_14 = line_comparator.compare(line_14_1, line_14_2);
        BOOST_CHECK_CLOSE_FRACTION(result_14.error_added, 6.0, 1e-6);
        BOOST_CHECK_EQUAL(result_14.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_14.error_fraction, 6.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_14.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_14.correct_fraction, 5.0 / 11, 1e-6);

        // reverse within long
        auto line_15_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_15_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 4, 0, 1, 0, 5})};
        auto result_15 = line_comparator.compare(line_15_1, line_15_2);
        BOOST_CHECK_CLOSE_FRACTION(result_15.error_added, 6.0, 1e-6);
        BOOST_CHECK_EQUAL(result_15.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_15.error_fraction, 6.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_15.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_15.correct_fraction, 5.0 / 11, 1e-6);

        // reverse missing
        auto line_16_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 2, 0, 1, 0, 5})};
        auto line_16_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto result_16 = line_comparator.compare(line_16_1, line_16_2);
        BOOST_CHECK_EQUAL(result_16.error_added, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_16.error_missed, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_16.error_fraction, 2.0 / 7, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_16.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_16.correct_fraction, 5.0 / 7, 1e-6);

        // reverse missing within
        auto line_17_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 2, 0, 1, 0, 5})};
        auto line_17_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 3, 0, 2, 0, 5})};
        auto result_17 = line_comparator.compare(line_17_1, line_17_2);
        BOOST_CHECK_CLOSE_FRACTION(result_17.error_added, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_17.error_missed, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_17.error_fraction, 4.0 / 7, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_17.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_17.correct_fraction, 5.0 / 7, 1e-6);

        // reverse correct
        auto line_18_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5, 0, 3, 0, 6})};
        auto line_18_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 4, 0, 5, 0, 3, 0, 4, 0, 6})
        };
        auto result_18 = line_comparator.compare(line_18_1, line_18_2);
        BOOST_CHECK_EQUAL(result_18.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_18.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_18.error_fraction, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_18.correct, 10.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_18.correct_fraction, 1.0, 1e-6);

        // reverse outside correct
        auto line_19_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 5, 1, 5, 1, 4, 0, 4, 0, 2, 1, 2, 1, 0})
        };
        auto line_19_2 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 5, 1, 5, 1, 4, 0, 4, 0, 3, 0, 2, 1, 2, 1, 0})
        };
        auto result_19 = line_comparator.compare(line_19_1, line_19_2);
        BOOST_CHECK_EQUAL(result_19.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_19.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_19.error_fraction, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_19.correct, 13.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_19.correct_fraction, 1.0, 1e-6);

        // reverse outside added
        auto line_20_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_20_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_20 = line_comparator.compare(line_20_1, line_20_2);
        BOOST_CHECK_CLOSE_FRACTION(result_20.error_added, 2.0, 1e-6);
        BOOST_CHECK_EQUAL(result_20.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_20.error_fraction, 2.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_20.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_20.correct_fraction, 5.0 / 7, 1e-6);

        // reverse added overlap
        auto line_21_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 1, 2, 1, 0, 1, 0, 5})};
        auto line_21_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 1, 3, 1, 0, 1, 0, 5})};
        auto result_21 = line_comparator.compare(line_21_1, line_21_2);
        BOOST_CHECK_CLOSE_FRACTION(result_21.error_added, 2.0, 1e-6);
        BOOST_CHECK_EQUAL(result_21.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_21.error_fraction, 2.0 / 9, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_21.correct, 9.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_21.correct_fraction, 9.0 / 11, 1e-6);

        // reverse start
        auto line_22_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_22_2 = rich_line_type{create_line<point_type, line_type>({0, 1, 0, 0, 0, 5})};
        auto result_22 = line_comparator.compare(line_22_1, line_22_2);
        BOOST_CHECK_CLOSE_FRACTION(result_22.error_added, 1.0, 1e-6);
        BOOST_CHECK_EQUAL(result_22.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_22.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_22.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_22.correct_fraction, 5.0 / 6, 1e-6);

        // reverse end
        auto line_23_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_23_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5, 0, 4})};
        auto result_23 = line_comparator.compare(line_23_1, line_23_2);
        BOOST_CHECK_CLOSE_FRACTION(result_23.error_added, 1.0, 1e-6);
        BOOST_CHECK_EQUAL(result_23.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_23.error_fraction, 1.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_23.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_23.correct_fraction, 5.0 / 6, 1e-6);

        // reverse both
        auto line_24_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5})};
        auto line_24_2 = rich_line_type{create_line<point_type, line_type>({0, 1, 0, 0, 0, 5, 0, 4})};
        auto result_24 = line_comparator.compare(line_24_1, line_24_2);
        BOOST_CHECK_CLOSE_FRACTION(result_24.error_added, 2.0, 1e-6);
        BOOST_CHECK_EQUAL(result_24.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_24.error_fraction, 2.0 / 5, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_24.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_24.correct_fraction, 5.0 / 7, 1e-6);

        // correct loop
        auto line_25_1 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_25_2 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_25 = line_comparator.compare(line_25_1, line_25_2);
        BOOST_CHECK_EQUAL(result_25.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_25.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_25.error_fraction, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_25.correct, 9.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_25.correct_fraction, 1.0, 1e-6);

        // loop outside
        auto line_26_1 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_26_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        0, 0, 0, 1, 0, 2, 1, 2, 2, 2, 2, 1, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5
                })
        };
        auto result_26 = line_comparator.compare(line_26_1, line_26_2);
        BOOST_CHECK_CLOSE_FRACTION(result_26.error_added, 3.0, 1e-6);
        BOOST_CHECK_EQUAL(result_26.error_missed, 1.0);
        BOOST_CHECK_CLOSE_FRACTION(result_26.error_fraction, 4.0 / 9, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_26.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_26.correct_fraction, 8.0 / 11, 1e-6);

        // loop outside reverse
        auto line_27_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 2, 1, 2, 1, 1, 0, 1, 0, 5})
        };
        auto line_27_2 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 2, 1, 2, 2, 2, 2, 1, 2, 2, 2, 1, 1, 1, 0, 1, 0, 5})
        };
        auto result_27 = line_comparator.compare(line_27_1, line_27_2);
        BOOST_CHECK_CLOSE_FRACTION(result_27.error_added, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_27.error_missed, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_27.error_fraction, 6.0 / 9, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_27.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_27.correct_fraction, 8.0 / 13, 1e-6);

        // loop reverse
        auto line_28_1 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_28_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 1, 2, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3,
                        0, 4, 0, 5
                })
        };
        auto result_28 = line_comparator.compare(line_28_1, line_28_2);
        BOOST_CHECK_CLOSE_FRACTION(result_28.error_added, 4.0, 1e-6);
        BOOST_CHECK_EQUAL(result_28.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_28.error_fraction, 4.0 / 9, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_28.correct, 9.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_28.correct_fraction, 9.0 / 13, 1e-6);

        // missing part
        auto line_29_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 1, 1, 1, 2, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_29_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_29 = line_comparator.compare(line_29_1, line_29_2);
        BOOST_CHECK_CLOSE_FRACTION(result_29.error_added, 1.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_29.error_missed, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_29.error_fraction, 4.0 / 7, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_29.correct, 4.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_29.correct_fraction, 4.0 / 7, 1e-6);

        // missing late
        auto line_30_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 1, 1, 1, 2, 1, 3, 1, 4, 1, 5})
        };
        auto line_30_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 1, 3, 1, 4, 1, 5})
        };
        auto result_30 = line_comparator.compare(line_30_1, line_30_2);
        BOOST_CHECK_CLOSE_FRACTION(result_30.error_added, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_30.error_missed, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_30.error_fraction, 6.0 / 6, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_30.correct, 3.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_30.correct_fraction, 3.0 / 6, 1e-6);

        // parallel same direction
        auto line_31_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_31_2 = rich_line_type{
                create_line<point_type, line_type>({1, 0, 1, 1, 1, 2, 1, 3, 1, 4, 1, 5})
        };
        auto result_31 = line_comparator.compare(line_31_1, line_31_2);
        BOOST_CHECK_CLOSE_FRACTION(result_31.error_added, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_31.error_missed, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_31.error_fraction, 10.0 / 5, 1e-6);
        BOOST_CHECK_EQUAL(result_31.correct, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_31.correct_fraction, 0.0 / 5, 1e-6);

        // parallel opposite direction
        auto line_32_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 1, 0, 3, 0, 4, 0, 5})};
        auto line_32_2 = rich_line_type{create_line<point_type, line_type>({1, 5, 1, 4, 1, 2, 1, 1, 1, 0})};
        auto result_32 = line_comparator.compare(line_32_1, line_32_2);
        BOOST_CHECK_CLOSE_FRACTION(result_32.error_added, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_32.error_missed, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_32.error_fraction, 10.0 / 5, 1e-6);
        BOOST_CHECK_EQUAL(result_32.correct, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_32.correct_fraction, 0.0 / 5, 1e-6);

        // ab zero
        auto line_33_1 = rich_line_type{line_type{}};
        auto line_33_2 = rich_line_type{line_type{}};
        auto result_33 = line_comparator.compare(line_33_1, line_33_2);
        BOOST_CHECK_EQUAL(result_33.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_33.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_33.error_fraction, 0.0);
        BOOST_CHECK_EQUAL(result_33.correct, 0.0);
        BOOST_CHECK_EQUAL(result_33.correct_fraction, 1.0);

        // a zero
        auto line_34_1 = rich_line_type{line_type{}};
        auto line_34_2 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto result_34 = line_comparator.compare(line_34_1, line_34_2);
        BOOST_CHECK_CLOSE_FRACTION(result_34.error_added, 5.0, 1e-6);
        BOOST_CHECK_EQUAL(result_34.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_34.error_fraction, std::numeric_limits<double>::infinity());
        BOOST_CHECK_EQUAL(result_34.correct, 0.0);
        BOOST_CHECK_EQUAL(result_34.correct_fraction, 0.0);

        // b zero
        auto line_35_1 = rich_line_type{
                create_line<point_type, line_type>({0, 0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_35_2 = rich_line_type{line_type{}};
        auto result_35 = line_comparator.compare(line_35_1, line_35_2);
        BOOST_CHECK_EQUAL(result_35.error_added, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_35.error_missed, 5.0, 1e-6);
        BOOST_CHECK_EQUAL(result_35.error_fraction, 1.0);
        BOOST_CHECK_EQUAL(result_35.correct, 0.0);
        BOOST_CHECK_EQUAL(result_35.correct_fraction, 0.0);

        // diagonal
        auto line_36_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 1, 0, 3, 2, 3, 3})};
        auto line_36_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 1, 0, 2, 1, 3, 2, 3, 3})};
        auto result_36 = line_comparator.compare(line_36_1, line_36_2);
        BOOST_CHECK_EQUAL(result_36.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_36.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_36.error_fraction, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_36.correct, line_36_1.length(), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_36.correct_fraction, 1.0, 1e-6);

        // almost equal
        auto line_37_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 5, 0})};
        auto line_37_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        -0.000000001, 0.000000002, 1.999999998, -0.000000001, 5.000000002,
                        0.000000001
                })
        };
        auto result_37 = line_comparator.compare(line_37_1, line_37_2);
        BOOST_CHECK_LE(result_37.error_added, 1e-6);
        BOOST_CHECK_LE(result_37.error_missed, 1e-6);
        BOOST_CHECK_LE(result_37.error_fraction, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_37.correct, 5.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_37.correct_fraction, 1.0, 1e-6);

        // diagonal almost equal
        auto line_38_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 1, 0, 3, 2, 3, 3})};
        auto line_38_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        0, 0, 1.000000002, 0.000000001, 2.000000001, 1.000000001, 3.000000002,
                        2.000000002, 3, 3
                })
        };
        auto result_38 = line_comparator.compare(line_38_1, line_38_2);
        BOOST_CHECK_EQUAL(result_38.error_added, 0.0);
        BOOST_CHECK_EQUAL(result_38.error_missed, 0.0);
        BOOST_CHECK_EQUAL(result_38.error_fraction, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_38.correct, line_38_1.length(), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_38.correct_fraction, 1.0, 1e-6);

        // reverse end turn
        auto line_39_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5, 0, 4})};
        auto line_39_2 = rich_line_type{create_line<point_type, line_type>({0, 0, 0, 5, 0, 4, 0, 5})};
        auto result_39 = line_comparator.compare(line_39_1, line_39_2);
        BOOST_CHECK_CLOSE_FRACTION(result_39.error_added, 1.0, 1e-6);
        BOOST_CHECK_EQUAL(result_39.error_missed, 0.0);
        BOOST_CHECK_CLOSE_FRACTION(result_39.error_fraction, 1.0 / 6, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_39.correct, 6.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_39.correct_fraction, 6.0 / 7, 1e-6);

        // early merged loop with late merge
        auto line_40_1 = rich_line_type{
                create_line<point_type, line_type>(
                        {0, -1, 0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5})
        };
        auto line_40_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        2, 2, 1, 2, 1, 1, 0, 1, 0, 0, 0, 1, 0, 2, 1, 2, 1, 1, 0, 1, 0, 2, 0, 3,
                        0, 4, 0, 5
                })
        };
        auto result_40 = line_comparator.compare(line_40_1, line_40_2);
        BOOST_CHECK_EQUAL(result_40.error_added, 4.0);
        BOOST_CHECK_EQUAL(result_40.error_missed, 1.0);
        BOOST_CHECK_CLOSE_FRACTION(result_40.error_fraction, 5.0 / 10, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_40.correct, 9.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_40.correct_fraction, 9.0 / 13, 1e-6);

        // almost equal reverse
        auto line_41_1 = rich_line_type{create_line<point_type, line_type>({0, 0, 5, 0, 2, 0, 3, 0})};
        auto line_41_2 = rich_line_type{
                create_line<point_type, line_type>(
                {
                        -0.000000001, 0.000000002, 5.000000002, 0.000000001, 1.999999998,
                        -0.000000001, 3.000000001,
                        0.000000002
                })
        };
        auto result_41 = line_comparator.compare(line_41_1, line_41_2);
        BOOST_CHECK_LE(result_41.error_added, 1e-6);
        BOOST_CHECK_LE(result_41.error_missed, 1e-6);
        BOOST_CHECK_LE(result_41.error_fraction, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_41.correct, line_41_1.length(), 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_41.correct_fraction, 1.0, 1e-6);

        // multi line equal
        auto line_42_1 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto line_42_2 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto result_42 = line_comparator.compare(line_42_1, line_42_2);
        BOOST_CHECK_LE(result_42.error_added, 1e-6);
        BOOST_CHECK_LE(result_42.error_missed, 1e-6);
        BOOST_CHECK_LE(result_42.error_fraction, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_42.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_42.correct_fraction, 1.0, 1e-6);

        // multi line not equal
        auto line_43_1 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto line_43_2 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 5, 0, 5, 5})}
                }
        };
        auto result_43 = line_comparator.compare(line_43_1, line_43_2);
        BOOST_CHECK_CLOSE_FRACTION(result_43.error_added, 2.0, 1e-6);
        BOOST_CHECK_LE(result_43.error_missed, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_43.error_fraction, 2.0 / 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_43.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_43.correct_fraction, 8.0 / 10.0, 1e-6);

        // multi line not equal exchanged
        auto line_44_1 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 5, 0, 5, 5})}
                }
        };
        auto line_44_2 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto result_44 = line_comparator.compare(line_44_1, line_44_2);
        BOOST_CHECK_LE(result_44.error_added, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_44.error_missed, 2.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_44.error_fraction, 2.0 / 10.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_44.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_44.correct_fraction, 8.0 / 10.0, 1e-6);

        // multi line almost equal
        auto line_45_1 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto line_45_2 = multi_rich_line_type{
                std::vector{
                        rich_line_type{
                                create_line<point_type, line_type>(
                                        {-0.000000001, 0.000000001, 4.000000002, 0.000000001})
                        },
                        rich_line_type{
                                create_line<point_type, line_type>(
                                        {4.999999998, 1.000000001, 4.999999999, 5.000000001})
                        }
                }
        };
        auto result_45 = line_comparator.compare(line_45_1, line_45_2);
        BOOST_CHECK_LE(result_45.error_added, 1e-6);
        BOOST_CHECK_LE(result_45.error_missed, 1e-6);
        BOOST_CHECK_LE(result_45.error_fraction, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_45.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_45.correct_fraction, 1.0, 1e-6);

        // multi line not almost equal
        auto line_46_1 = multi_rich_line_type{
                std::vector{
                        rich_line_type{create_line<point_type, line_type>({0, 0, 4, 0})},
                        rich_line_type{create_line<point_type, line_type>({5, 1, 5, 5})}
                }
        };
        auto line_46_2 = multi_rich_line_type{
                std::vector{
                        rich_line_type{
                                create_line<point_type, line_type>(
                                {
                                        -0.000000001, 0.000000001, 5.000000002, 0.000000001,
                                        4.999999998, 0.000000001, 4.999999999, 5.000000001
                                })
                        }
                }
        };
        auto result_46 = line_comparator.compare(line_46_1, line_46_2);
        BOOST_CHECK_CLOSE_FRACTION(result_46.error_added, 2.0, 1e-6);
        BOOST_CHECK_LE(result_46.error_missed, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_46.error_fraction, 2.0 / 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_46.correct, 8.0, 1e-6);
        BOOST_CHECK_CLOSE_FRACTION(result_46.correct_fraction, 8.0 / 10.0, 1e-6);
    }

BOOST_AUTO_TEST_SUITE_END()
