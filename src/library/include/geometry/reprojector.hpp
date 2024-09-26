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

#ifndef MAP_MATCHING_2_GEOMETRY_REPROJECTOR_HPP
#define MAP_MATCHING_2_GEOMETRY_REPROJECTOR_HPP

#include "srs.hpp"

namespace map_matching_2::geometry {

    template<typename InPoint, typename OutPoint>
    class point_reprojector {

    public:
        using in_point_type = InPoint;
        using out_point_type = OutPoint;

        explicit point_reprojector(const srs_transform &srs_transform)
            : _transform{srs_transform.transform}, _transformation{create_transformation(srs_transform)} {}

        [[nodiscard]] out_point_type operator()(const in_point_type &in) const {
            out_point_type out;
            _transformation.forward(in, out);
            return out;
        }

        [[nodiscard]] constexpr bool transform() const {
            return _transform;
        }

    private:
        bool _transform;
        boost::geometry::srs::transformation<> _transformation{};

    };

}

#endif //MAP_MATCHING_2_GEOMETRY_REPROJECTOR_HPP
