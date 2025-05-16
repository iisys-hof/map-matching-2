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

#ifndef MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP
#define MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP

#include "geometry/algorithm/split.hpp"

namespace map_matching_2::io::track::filter {

    template<typename Forwarder>
    class track_time_splitter {

    public:
        using forwarder_type = Forwarder;
        using import_multi_track_variant_type = typename forwarder_type::import_multi_track_variant_type;

        constexpr explicit track_time_splitter(forwarder_type &forwarder, const double split_time,
                const std::string &aggregator)
            : _forwarder{forwarder}, _split_time{split_time}, _aggregator{aggregator}, _no_split{split_time <= 0} {}

        void pass(import_multi_track_variant_type multi_track) {
            if (_no_split) {
                _forwarder.pass(std::move(multi_track));
            } else {
                _split_track(std::move(multi_track));
            }
        }

    protected:
        forwarder_type &_forwarder;
        const double _split_time;
        const std::string _aggregator;
        const bool _no_split{false};

        void _split_track(import_multi_track_variant_type &&multi_track) {
            std::visit([this]<typename ImportMultiTrack>(ImportMultiTrack &&multi_track) {
                using multi_track_type = std::remove_reference_t<ImportMultiTrack>;
                using multi_rich_line_type = typename multi_track_type::multi_rich_line_type;

                bool called = false;
                std::size_t index{0};
                const auto &multi_rich_line = multi_track.multi_rich_line;

                geometry::multi_rich_line_time_split<multi_rich_line_type>(multi_rich_line, _split_time,
                        [&](const std::size_t from_outer, const std::size_t from_inner,
                        const std::size_t to_inner, const std::size_t to_outer) {
                            called = true;
                            _forwarder.pass(std::forward<multi_track_type>(multi_track_type{
                                    std::format("{}{}{}", multi_track.id, _aggregator, index++),
                                    multi_rich_line.template extract<multi_rich_line_type>(
                                            from_outer, from_inner, to_inner, to_outer)
                            }));
                        });

                if (not called) {
                    _forwarder.pass(std::forward<multi_track_type>(multi_track));
                }
            }, std::move(multi_track));
        }

    };

}

#endif //MAP_MATCHING_2_IO_TRACK_FILTER_TRACK_TIME_SPLITTER_HPP
