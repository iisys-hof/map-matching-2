// Copyright (C) 2021-2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_IO_TRACK_INPUT_IMPORTER_HPP
#define MAP_MATCHING_2_IO_TRACK_INPUT_IMPORTER_HPP

#include <format>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include <boost/geometry/io/wkt/read.hpp>

#include <boost/tokenizer.hpp>

#include "types/geometry/track/multi_track.hpp"
#include "types/geometry/reprojector.hpp"

#include "geometry/algorithm/convert.hpp"
#include "geometry/algorithm/srs_dispatch.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class input_importer {

    public:
        using forwarder_type = Forwarder;

        using multi_track_variant_type = typename forwarder_type::multi_track_variant_type;

        constexpr input_importer(forwarder_type &forwarder, const geometry::srs_transform &srs_transform,
                const geometry::point_reprojector_variant &reprojector_variant)
            : _forwarder{forwarder}, _srs_transform{srs_transform}, _reprojector_variant{reprojector_variant} {}

        void read() {
            std::string wkt_string;
            while (std::getline(std::cin, wkt_string)) {
                boost::trim(wkt_string);
                if (wkt_string.empty()) {
                    break;
                }

                boost::to_upper(wkt_string);

                try {
                    _parse_dispatch(wkt_string);
                } catch (...) {
                    std::clog << std::format("Could not parse the following track, "
                            "only POINT, LINESTRING, and MULTILINESTRING WKT strings are supported, "
                            "skipping ...\n{}", wkt_string) << std::endl;
                }
            }
        }

    protected:
        forwarder_type &_forwarder;
        const geometry::srs_transform &_srs_transform;
        const geometry::point_reprojector_variant &_reprojector_variant;

        boost::char_separator<char> _separator{",;|"};
        std::size_t _counter{0};

        void _parse_dispatch(const std::string &wkt_string) {
            geometry::srs_dispatch(_srs_transform, [this, &wkt_string]<typename InputCS, typename OutputCS>() {
                using point_type_in = geometry::point_type<InputCS>;
                using point_type_out = geometry::point_type<OutputCS>;

                using line_type_in = typename geometry::models<point_type_in>::template line_type<>;
                using multi_line_type_in = typename geometry::models<point_type_in>::
                        template multi_line_type<line_type_in>;

                using multi_track_type = geometry::track::multi_track_type<point_type_out>;
                using measurement_type_out = typename multi_track_type::measurement_type;
                using line_type_out = typename multi_track_type::line_type;

                std::string id_str = std::to_string(_counter++);

                if (wkt_string.starts_with("POINT")) {
                    std::vector<measurement_type_out> measurements;
                    std::uint64_t timestamp = 0;
                    boost::tokenizer tokenizer{wkt_string, _separator};
                    for (auto wkt_point : tokenizer) {
                        boost::trim(wkt_point);
                        point_type_in point;
                        boost::geometry::read_wkt(wkt_point, point);
                        measurements.emplace_back(measurement_type_out{
                                timestamp++, std::move(point), _reprojector_variant
                        });
                    }

                    _forwarder.pass(multi_track_type{
                            id_str, geometry::measurements2line<measurement_type_out, line_type_out>(
                                    std::move(measurements))
                    });
                } else if (wkt_string.starts_with("LINESTRING")) {
                    line_type_in line;
                    boost::geometry::read_wkt(wkt_string, line);
                    _forwarder.pass(multi_track_type{id_str, std::move(line), _reprojector_variant});
                } else if (wkt_string.starts_with("MULTILINESTRING")) {
                    multi_line_type_in multi_line;
                    boost::geometry::read_wkt(wkt_string, multi_line);
                    _forwarder.pass(multi_track_type{id_str, std::move(multi_line), _reprojector_variant});
                } else {
                    throw std::exception{};
                }
            });
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_INPUT_IMPORTER_HPP
