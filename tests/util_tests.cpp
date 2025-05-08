// Copyright (C) 2025 Adrian Wöltche
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

#include <chrono>
#include <boost/test/unit_test.hpp>

#include "util/chrono.hpp"

BOOST_AUTO_TEST_SUITE(util_tests)

    BOOST_AUTO_TEST_CASE(parse_time_test) {
        const std::string time_format = "%FT%T%Oz";

        std::string time_string = "2025-01-02T03:04:05+01:00";
        auto time = map_matching_2::util::parse_time(time_string, time_format);
        BOOST_CHECK_EQUAL(time, 1735783445);

        time_string = "2025-01-02T03:04:06+01:00";
        time = map_matching_2::util::parse_time(time_string, time_format);
        BOOST_CHECK_EQUAL(time, 1735783446);
    }

    BOOST_AUTO_TEST_CASE(format_time_test) {
        const std::string locale = "UTC";

        std::uint64_t time = 1735783445;
        auto time_string = map_matching_2::util::format_time(time, locale);
        BOOST_CHECK_EQUAL(time_string, "2025-01-02 02:04:05 UTC");

        time = 1735783446;
        time_string = map_matching_2::util::format_time(time, locale);
        BOOST_CHECK_EQUAL(time_string, "2025-01-02 02:04:06 UTC");
    }

BOOST_AUTO_TEST_SUITE_END()
