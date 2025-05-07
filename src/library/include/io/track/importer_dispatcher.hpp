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

#ifndef MAP_MATCHING_2_IO_TRACK_IMPORTER_DISPATCHER_HPP
#define MAP_MATCHING_2_IO_TRACK_IMPORTER_DISPATCHER_HPP

#include "types/io/track/csv_track_importer.hpp"
#include "types/io/track/gpx_track_importer.hpp"

#include "importer_settings.hpp"

namespace map_matching_2::io::track {

    template<typename Forwarder>
    class importer_dispatcher {

    public:
        importer_dispatcher(Forwarder &forwarder, importer_settings importer_settings,
                const geometry::point_reprojector_variant &reprojector_variant)
            : _forwarder{forwarder}, _settings{std::move(importer_settings)},
            _reprojector_variant{reprojector_variant} {}

        void read_tracks() {
            if (_settings.file_extension == importer_settings::FILE_TYPE::CSV) {
                _read_csv();
            } else if (_settings.file_extension == importer_settings::FILE_TYPE::GPX) {
                _read_gpx();
            }
        }

    private:
        Forwarder &_forwarder;
        importer_settings _settings;
        const geometry::point_reprojector_variant &_reprojector_variant;

        void _read_csv() {
            io::track::csv_track_importer<Forwarder> csv_track_importer{
                    _settings.files, _settings.csv_settings,
                    _forwarder, _settings.tracks_srs_transform, _reprojector_variant
            };
            csv_track_importer.read();
        }

        void _read_gpx() {
            io::track::gpx_track_importer<Forwarder> gpx_track_importer{
                    _settings.files, _settings.csv_settings.selectors, _settings.csv_settings.no_id,
                    _settings.csv_settings.time_format, _settings.csv_settings.no_parse_time,
                    _forwarder, _settings.tracks_srs_transform, _reprojector_variant
            };
            gpx_track_importer.read();
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_IMPORTER_DISPATCHER_HPP
