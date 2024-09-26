// Copyright (C) 2020-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_GEOMETRY_SRS_HPP
#define MAP_MATCHING_2_GEOMETRY_SRS_HPP

#include <cstdint>

#include <boost/geometry/srs/projections/epsg_params.hpp>
#include <boost/geometry/srs/transformation.hpp>

namespace map_matching_2::geometry {

    enum SRS {
        CS_INVALID,
        CS_GEOGRAPHIC,
        CS_SPHERICAL_EQUATORIAL,
        CS_CARTESIAN
    };

    std::string to_string(const SRS &srs);

    struct srs {
        std::int32_t srs_code{-1};
        SRS srs_type = SRS::CS_INVALID;
        boost::geometry::srs::epsg proj{-1};

        constexpr srs() = default;

        srs(std::int32_t srs_code, SRS srs_type);

        srs(std::int32_t srs_code, SRS srs_type, boost::geometry::srs::epsg proj);
    };

    struct srs_transform : srs {
        std::int32_t transform_srs_code{-1};
        SRS transform_srs_type = SRS::CS_INVALID;
        boost::geometry::srs::epsg transform_proj{-1};
        bool transform{false};

        constexpr srs_transform() = default;

        srs_transform(srs base_srs, std::int32_t transform_srs_code, SRS transform_srs_type);

        srs_transform(srs base_srs, std::int32_t transform_srs_code, SRS transform_srs_type,
                boost::geometry::srs::epsg transform_proj);
    };

    [[nodiscard]] srs compute_srs(std::int32_t srs_code, const std::string &srs_type);

    [[nodiscard]] srs_transform compute_srs_transform(std::int32_t srs_code, const std::string &srs_type,
            std::int32_t transform_srs_code, const std::string &transform_srs_type);

    [[nodiscard]] srs_transform compute_next_srs_transform(const srs_transform &_srs_transform,
            std::int32_t transform_srs_code, SRS transform_srs_type);

    [[nodiscard]] boost::geometry::srs::transformation<> create_transformation(const srs_transform &_srs_transform);

}

#endif //MAP_MATCHING_2_GEOMETRY_SRS_HPP
