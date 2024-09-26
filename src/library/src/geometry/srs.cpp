// Copyright (C) 2022-2024 Adrian Wöltche
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

#include <format>

#include <boost/geometry/srs/epsg.hpp>

#include "geometry/srs.hpp"

namespace map_matching_2::geometry {

    static const std::string CS_GEOGRAPHIC_STRING{"geographic"};
    static const std::string CS_SPHERICAL_EQUATORIAL_STRING{"spherical-equatorial"};
    static const std::string CS_CARTESIAN_STRING{"cartesian"};

    SRS _get_srs_type(const std::string &srs_type) {
        if (srs_type == CS_GEOGRAPHIC_STRING) {
            return SRS::CS_GEOGRAPHIC;
        } else if (srs_type == CS_SPHERICAL_EQUATORIAL_STRING) {
            return SRS::CS_SPHERICAL_EQUATORIAL;
        } else if (srs_type == CS_CARTESIAN_STRING) {
            return SRS::CS_CARTESIAN;
        } else {
            return SRS::CS_INVALID;
        }
    }

    SRS _get_srs_type(const boost::geometry::srs::epsg &proj) {
        const boost::geometry::projections::proj_wrapper<boost::geometry::srs::dynamic, double> proj_wrapper{proj};
        const auto &params = proj_wrapper.proj().params();
        const bool is_geographic = not params.is_geocent and params.is_latlong;
        if (is_geographic) {
            return SRS::CS_GEOGRAPHIC;
        } else {
            return SRS::CS_CARTESIAN;
        }
    }

    std::string to_string(const SRS &srs) {
        switch (srs) {
            case SRS::CS_GEOGRAPHIC:
                return "GEOGRAPHIC";
            case SRS::CS_SPHERICAL_EQUATORIAL:
                return "SPHERICAL_EQUATORIAL";
            case SRS::CS_CARTESIAN:
                return "CARTESIAN";
            default:
                return "INVALID";
        }
    }

    srs::srs(const std::int32_t srs_code, const SRS srs_type)
        : srs{srs_code, srs_type, boost::geometry::srs::epsg{srs_code}} {}

    srs::srs(const std::int32_t srs_code, const SRS srs_type, const boost::geometry::srs::epsg proj)
        : srs_code{srs_code}, srs_type{srs_type}, proj{proj} {}

    srs_transform::srs_transform(srs base_srs, const std::int32_t transform_srs_code,
            const SRS transform_srs_type)
        : srs_transform{
                std::move(base_srs),
                transform_srs_code, transform_srs_type, boost::geometry::srs::epsg{transform_srs_code}
        } {}

    srs_transform::srs_transform(srs base_srs, const std::int32_t transform_srs_code,
            const SRS transform_srs_type, const boost::geometry::srs::epsg transform_proj)
        : srs{std::move(base_srs)},
        transform_srs_code{transform_srs_code}, transform_srs_type{transform_srs_type}, transform_proj{transform_proj} {
        transform = not(proj.code == transform_proj.code and srs_type == transform_srs_type);
    }

    srs compute_srs(const std::int32_t srs_code, const std::string &srs_type) {
        // do not accept invalid default srs
        if (srs_code < 0) {
            throw std::invalid_argument{std::format("invalid srs: {}", srs_code)};
        }

        const boost::geometry::srs::epsg proj{srs_code};

        SRS cs_srs = _get_srs_type(srs_type);
        if (cs_srs == SRS::CS_INVALID) {
            cs_srs = _get_srs_type(proj);
        }

        return srs{srs_code, cs_srs, proj};
    }

    srs_transform compute_srs_transform(const std::int32_t srs_code, const std::string &srs_type,
            std::int32_t transform_srs_code, const std::string &transform_srs_type) {
        const srs base_srs = compute_srs(srs_code, srs_type);

        boost::geometry::srs::epsg transform_proj{transform_srs_code};

        SRS cs_transform_srs = _get_srs_type(transform_srs_type);
        if (cs_transform_srs == SRS::CS_GEOGRAPHIC and transform_srs_code < 0) {
            transform_srs_code = 4326;
            transform_proj = boost::geometry::srs::epsg{transform_srs_code};
        } else if (cs_transform_srs == SRS::CS_CARTESIAN and transform_srs_code < 0) {
            transform_srs_code = 3857;
            transform_proj = boost::geometry::srs::epsg{transform_srs_code};
        } else if (cs_transform_srs != SRS::CS_INVALID and transform_srs_code < 0) {
            transform_srs_code = srs_code;
            transform_proj = boost::geometry::srs::epsg{transform_srs_code};
        } else if (cs_transform_srs == SRS::CS_INVALID and transform_srs_code >= 0) {
            cs_transform_srs = _get_srs_type(transform_proj);
        }

        if (cs_transform_srs == SRS::CS_INVALID) {
            // still invalid crs, so no transformation
            transform_srs_code = base_srs.srs_code;
            cs_transform_srs = base_srs.srs_type;
            transform_proj = base_srs.proj;
        }

        return srs_transform{base_srs, transform_srs_code, cs_transform_srs, transform_proj};
    }

    srs_transform compute_next_srs_transform(const srs_transform &_srs_transform, const std::int32_t transform_srs_code,
            const SRS transform_srs_type) {

        if (_srs_transform.transform_srs_type != SRS::CS_INVALID and _srs_transform.srs_code != transform_srs_code and
            _srs_transform.transform_srs_type != transform_srs_type) {
            return srs_transform{
                    srs{_srs_transform.srs_code, _srs_transform.transform_srs_type},
                    transform_srs_code, transform_srs_type
            };
        }

        return srs_transform{
                srs{transform_srs_code, transform_srs_type},
                transform_srs_code, transform_srs_type
        };
    }

    boost::geometry::srs::transformation<> create_transformation(const srs_transform &_srs_transform) {
        return boost::geometry::srs::transformation<>{_srs_transform.proj, _srs_transform.transform_proj};
    }

}
