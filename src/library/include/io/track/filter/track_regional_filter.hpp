// Copyright (C) 2024-2025 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_REGIONAL_FILTER_HPP
#define MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_REGIONAL_FILTER_HPP

#include <string>
#include <format>
#include <variant>

#include <boost/geometry/io/wkt/read.hpp>

#include "types/geometry/point.hpp"

#include "geometry/types.hpp"

namespace map_matching_2::io::track::filter {

    enum FILTER_METHOD {
        NONE,
        WITHIN,
        INTERSECTS
    };

    template<typename Forwarder>
    class track_regional_filter {

    public:
        using forwarder_type = Forwarder;
        using import_multi_track_variant_type = typename forwarder_type::import_multi_track_variant_type;

        constexpr explicit track_regional_filter(forwarder_type &forwarder,
                const std::string &polygon, const FILTER_METHOD method)
            : _forwarder{forwarder}, _polygon{polygon}, _method{method}, _empty{polygon.empty()} {}

        void pass(import_multi_track_variant_type multi_track) {
            if (_empty or _method == FILTER_METHOD::NONE) {
                _forwarder.pass(std::move(multi_track));
            } else {
                _filter_track(std::move(multi_track));
            }
        }

    protected:
        forwarder_type &_forwarder;
        const std::string _polygon;
        const FILTER_METHOD _method;
        const bool _empty{false};
        std::variant<
            std::monostate,
            geometry::models<geometry::point_type<geometry::cs_geographic>>::polygon_type<>,
            geometry::models<geometry::point_type<geometry::cs_spherical_equatorial>>::polygon_type<>,
            geometry::models<geometry::point_type<geometry::cs_cartesian>>::polygon_type<>> _polygon_variant;
        std::variant<
            std::monostate,
            geometry::models<geometry::point_type<geometry::cs_geographic>>::box_type,
            geometry::models<geometry::point_type<geometry::cs_spherical_equatorial>>::box_type,
            geometry::models<geometry::point_type<geometry::cs_cartesian>>::box_type> _box_variant;

        void _filter_track(import_multi_track_variant_type &&multi_track) {
            std::visit([this]<typename ImportMultiTrack>(ImportMultiTrack &&multi_track) {
                using multi_track_type = std::remove_reference_t<ImportMultiTrack>;
                using time_point_type = typename multi_track_type::point_type;
                using point_type = typename geometry::models<time_point_type>::explicit_point_type;

                if (_polygon.starts_with("BOX")) {
                    using box_type = typename geometry::models<point_type>::box_type;
                    if (std::holds_alternative<std::monostate>(_box_variant)) {
                        box_type box;
                        boost::geometry::read_wkt(_polygon, box);
                        _box_variant.emplace<box_type>(box);
                    }
                    const box_type &box = std::get<box_type>(_box_variant);
                    if (_method == WITHIN and
                        boost::geometry::covered_by(multi_track.multi_rich_line.multi_line(), box)) {
                        _forwarder.pass(std::forward<multi_track_type>(multi_track));
                    } else if (_method == INTERSECTS and
                        boost::geometry::intersects(multi_track.multi_rich_line.multi_line(), box)) {
                        _forwarder.pass(std::forward<multi_track_type>(multi_track));
                    }
                } else if (_polygon.starts_with("POLYGON")) {
                    using polygon_type = typename geometry::models<point_type>::template polygon_type<>;
                    if (std::holds_alternative<std::monostate>(_polygon_variant)) {
                        polygon_type polygon;
                        boost::geometry::read_wkt(_polygon, polygon);
                        _polygon_variant.emplace<polygon_type>(polygon);
                    }
                    const polygon_type &polygon = std::get<polygon_type>(_polygon_variant);
                    if (_method == WITHIN and
                        boost::geometry::covered_by(multi_track.multi_rich_line.multi_line(), polygon)) {
                        _forwarder.pass(std::forward<multi_track_type>(multi_track));
                    } else if (_method == INTERSECTS and
                        boost::geometry::intersects(multi_track.multi_rich_line.multi_line(), polygon)) {
                        _forwarder.pass(std::forward<multi_track_type>(multi_track));
                    }
                } else {
                    throw std::invalid_argument{std::format("invalid filter polygon: {}", _polygon)};
                }
            }, std::move(multi_track));
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_REGIONAL_FILTER_HPP
