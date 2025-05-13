// Copyright (C) 2024 Adrian Wöltche
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

#ifndef MAP_MATCHING_2_APP_MATCH_HPP
#define MAP_MATCHING_2_APP_MATCH_HPP

#include <memory>

#include "global.hpp"

#include "options.hpp"
#include "network.hpp"
#include "settings.hpp"

#include "types/io/track/input_importer.hpp"
#include "types/io/track/edges_list_importer.hpp"

#include "io/track/filter/track_time_splitter.hpp"
#include "io/track/filter/track_regional_filter.hpp"

#include "io/track/importer_dispatcher.hpp"
#include "io/track/track_exporter.hpp"

namespace map_matching_2::app {

    template<typename Network>
    [[nodiscard]] std::unique_ptr<matching::matcher<Network>> _start_matcher(
            std::unique_ptr<Network> &network, const match_data &data) {
        const auto output_file = std::filesystem::path{data.match_output.output} / data.match_output.filename;
        const auto output_candidates_file = std::filesystem::path{data.match_output.output} / data.expo.candidates;
        const auto output_filename = output_file.string();
        const auto output_candidates_filename = output_candidates_file.string();

        const matching::matcher_output_settings _output_settings{
                output_filename, data.match_output.columns,
                output_candidates_filename, data.expo.candidates_columns,
                data.console.console, global.verbose
        };

        auto matcher = std::make_unique<matching::matcher<Network>>(
                *network, _output_settings, data.performance.threads);
        matcher->start();
        return matcher;
    }

    template<typename Matcher>
    void _join_matcher(std::unique_ptr<Matcher> &matcher) {
        matcher->join();
        matcher.reset();
    }

    template<typename Network>
    void _open_network(std::unique_ptr<Network> &network, const match_data &data) {
        if (not data.memory.mmap_tags or not data.memory.mmap_indices or not data.memory.mmap_graph) {
            verbose_frame("Deserialize", [&]() {
                load_network(*network, data);
            });
        }
    }

    template<typename Forwarder>
    void _read_line(Forwarder &forwarder, const match_data &data,
            const geometry::point_reprojector_variant &reprojector_variant) {
        if (not global.quiet and not data.console.console) {
            std::cout << "Please input tracks one per line, "
                    "either as LINESTRING or comma separated sequence of POINT "
                    "(enter empty line for stopping):" << std::endl;
        }

        io::track::input_importer<Forwarder> input_importer{
                forwarder, data.srs_tracks.srs_transform, reprojector_variant
        };
        input_importer.read();
    }

    template<typename Graph>
    void _read_edge_list(const Graph &graph, const match_data &data,
            const geometry::point_reprojector_variant &reprojector_variant) {
        const auto output_file = std::filesystem::path{data.match_output.output} / data.match_output.filename;
        const auto output_filename = output_file.string();

        io::track::track_exporter track_exporter{output_filename, data.console.console, global.verbose};
        io::track::edges_list_importer<io::track::track_exporter, Graph> edges_list_importer{
                data.tracks.files, track_exporter, data.srs_tracks.srs_transform, reprojector_variant, graph,
                data.csv.delimiter, data.csv.ids, data.csv.skip_lines
        };
        edges_list_importer.read();
    }

    void _read_seattle(const match_data &data, const geometry::point_reprojector_variant &reprojector_variant);

    template<typename Forwarder, typename Matcher>
    void _read_tracks_forward(Forwarder &forwarder, std::unique_ptr<Matcher> &matcher, const match_data &data,
            const geometry::point_reprojector_variant &reprojector_variant) {
        if (data.console.read_line) {
            _read_line(forwarder, data, reprojector_variant);
        } else {
            if (data.tracks.file_extension == FILE_TYPE_LIST) {
                _read_edge_list(matcher->network().graph(), data, reprojector_variant);
            } else if (data.tracks.file_extension == FILE_TYPE_SEATTLE) {
                _read_seattle(data, reprojector_variant);
            } else {
                // regular track file types
                io::track::importer_dispatcher importer_dispatcher{
                        forwarder, _importer_settings(data), reprojector_variant
                };
                importer_dispatcher.read_tracks();
            }
        }
    }

    io::track::filter::FILTER_METHOD get_filter_method(const std::string &method);

    template<typename Matcher>
    void _read_tracks(std::unique_ptr<Matcher> &matcher, const match_data &data) {
        const auto reprojector_variant = geometry::create_point_reprojector(data.srs_tracks.srs_transform);

        matching::matcher_forwarder<Matcher> forwarder{*matcher, _match_settings(data)};
        io::track::filter::track_regional_filter regional_filter{
                forwarder, data.matching.filter_polygon, get_filter_method(data.matching.filter_method)
        };
        io::track::filter::track_time_splitter time_splitter{
                regional_filter, data.matching.split_time, data.csv.id_aggregator
        };

        _read_tracks_forward(time_splitter, matcher, data, reprojector_variant);

        // reading finished, close queue
        matcher->stop();
    }

    template<typename Network>
    void _start_matching(std::unique_ptr<Network> &network, const match_data &data) {
        _open_network<Network>(network, data);
        auto matcher = _start_matcher(network, data);
        _read_tracks(matcher, data);
        _join_matcher(matcher);
    }

}

#endif //MAP_MATCHING_2_APP_MATCH_HPP
