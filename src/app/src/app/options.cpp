// Copyright (C) 2022-2025 Adrian Wöltche
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

#include "app/options.hpp"
#include "app/global.hpp"

#include <format>
#include <thread>
#include <iostream>
#include <filesystem>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/unordered/unordered_flat_set.hpp>

#include "io/network/osm_pattern.hpp"
#include "compare/comparator/compare_output_settings.hpp"
#include "matching/matcher/matcher_output_settings.hpp"
#include "util/chrono.hpp"
#include "util/version.hpp"

namespace map_matching_2::app {

    namespace po = boost::program_options;

    void base_data::correct(const po::variables_map &vm) {}

    void all_data::correct(const po::variables_map &vm) {
        if (quiet) {
            verbose = false;
        }
    }

    void global_data::correct(const po::variables_map &vm) {
        std::size_t counter{0};

        if (prepare) {
            mode = MODE::PREPARE;
            counter++;
        }
        if (match) {
            mode = MODE::MATCH;
            counter++;
        }
        if (server) {
            mode = MODE::SERVER;
            counter++;
        }
        if (compare) {
            mode = MODE::COMPARE;
            counter++;
        }

        if (counter != 1) {
            throw std::invalid_argument{"please specify exactly one mode per application run."};
        }
    }

    void input_data::correct(const po::variables_map &vm) {
        if (not std::filesystem::is_directory(input)) {
            throw std::invalid_argument{"input path does not point to an existing directory."};
        }
    }

    void match_output_data::correct(const po::variables_map &vm) {
        std::vector<std::string> _column_strings;
        boost::split(_column_strings, columns, boost::is_any_of(","));
        export_edge_ids = std::find(std::cbegin(_column_strings), std::cend(_column_strings), "edge_ids") !=
                std::cend(_column_strings);

        boost::trim(export_time_zone);
        if (export_time_zone.empty()) {
            export_time_zone = "UTC";
        }

        if (not vm["quiet"].empty() and not vm["quiet"].as<bool>() and
            not vm["console"].empty() and not vm["console"].as<bool>()) {
            if (export_edges) {
                std::cout << "Enabled export edges mode." << std::endl;
            }
            if (not join_merges) {
                std::cout << "Disabled joining merged routes." << std::endl;
            }
        }
    }

    void prepare_network_data::correct(const po::variables_map &vm) {
        if (format != FORMAT_OSM and format != FORMAT_ARC_NODE and format != FORMAT_SEATTLE and
            format != FORMAT_MELBOURNE and format != FORMAT_GISCUP) {
            throw std::invalid_argument{std::format("data format {} is unknown.", format)};
        }
    }

    void prepare_osm_import_data::correct(const po::variables_map &vm) {
        if (profile != OSM_TAGS_PROFILE_CAR and profile != OSM_TAGS_PROFILE_MANUAL) {
            throw std::invalid_argument{std::format("osm-profile {} is unknown.", profile)};
        }

        if (profile == OSM_TAGS_PROFILE_MANUAL) {
            if (not std::regex_match(query, io::network::OSM_TAGS_FILTER_PATTERN)) {
                throw std::invalid_argument{
                        std::format("osm-query {} does not match the filter specification", query)
                };
            }
            if (not std::regex_match(filter, io::network::OSM_TAGS_FILTER_PATTERN)) {
                throw std::invalid_argument{
                        std::format("osm-filter {} does not match the filter specification", filter)
                };
            }
            if (not std::regex_match(oneway, io::network::OSM_TAGS_FILTER_PATTERN)) {
                throw std::invalid_argument{
                        std::format("osm-oneway {} does not match the filter specification", oneway)
                };
            }
            if (not std::regex_match(oneway_reverse, io::network::OSM_TAGS_FILTER_PATTERN)) {
                throw std::invalid_argument{
                        std::format("osm-oneway-reverse {} does not match the filter specification", oneway_reverse)
                };
            }
        }
    }

    void srs_data::correct(const po::variables_map &vm) {
        srs = geometry::compute_srs(_srs, _srs_type);
    }

    void srs_transform_data::correct(const po::variables_map &vm) {
        srs_transform = geometry::compute_srs_transform(_srs, _srs_type, _transform_srs, _transform_srs_type);
    }

    void files_data::correct(const po::variables_map &vm) {
        boost::unordered_flat_set<std::string> _files{};

        if (graph_file.empty()) {
            throw std::invalid_argument{"graph-file is not specified."};
        }
        if (index_file.empty()) {
            throw std::invalid_argument{"index-file is not specified."};
        }
        if (tags_file.empty()) {
            throw std::invalid_argument{"tags-file is not specified."};
        }

        _files.emplace(graph_file);
        _files.emplace(index_file);
        _files.emplace(tags_file);

        if (_files.size() != 3) {
            throw std::invalid_argument{"graph-file, index-file, and tags-file need to be individual filenames."};
        }
    }

    void prepare_process_data::correct(const po::variables_map &vm) {
        if (simplify != SIMPLIFY_NONE and simplify != SIMPLIFY_ALL and simplify != SIMPLIFY_OSM_AWARE) {
            throw std::invalid_argument{std::format("simplify specification {} is unknown.", simplify)};
        }

        const std::string format = vm["format"].as<std::string>();
        if (format == FORMAT_SEATTLE) {
            const bool is_remove_duplicate_nodes_manually = not vm["remove-duplicate-nodes"].defaulted();
            if (not is_remove_duplicate_nodes_manually) {
                remove_duplicate_nodes = true;
                std::cout << "Automatically enabled remove-duplicate-nodes." << std::endl;
            }
        }
    }

    void memory_data::correct(const po::variables_map &vm) {
        const bool is_memory_mapping_manually = not vm["memory-mapping"].defaulted();
        const bool is_mmap_graph_manually = not vm["memory-mapped-graph"].defaulted();
        const bool is_mmap_indices_manually = not vm["memory-mapped-indices"].defaulted();
        const bool is_mmap_tags_manually = not vm["memory-mapped-tags"].defaulted();

        if (is_memory_mapping_manually) {
            if (not is_mmap_graph_manually) {
                mmap_graph = memory_mapping;
            }
            if (not is_mmap_indices_manually) {
                mmap_indices = memory_mapping;
            }
            if (not is_mmap_tags_manually) {
                mmap_tags = memory_mapping;
            }
        }
    }

    void prepare_memory_data::correct(const po::variables_map &vm) {
        memory_data::correct(vm);

        const bool is_memory_mapping_manually = not vm["memory-mapping"].defaulted();
        bool is_mmap_preparation_manually = not vm["memory-mapped-preparation"].defaulted();
        const bool is_mmap_graph_import_manually = not vm["memory-mapped-graph-import"].defaulted();
        const bool is_mmap_tags_import_manually = not vm["memory-mapped-tags-import"].defaulted();
        const bool is_mmap_index_build_manually = not vm["memory-mapped-index-build"].defaulted();
        const bool is_mmap_osm_vertices_manually = not vm["memory-mapped-osm-vertices"].defaulted();
        const bool is_mmap_osm_tags_manually = not vm["memory-mapped-osm-tags"].defaulted();

        if (is_memory_mapping_manually) {
            if (not is_mmap_preparation_manually) {
                mmap_preparation = memory_mapping;
                is_mmap_preparation_manually = true;
            }
        }

        if (is_mmap_preparation_manually) {
            if (not is_mmap_graph_import_manually) {
                mmap_graph_import = mmap_preparation;
            }
            if (not is_mmap_tags_import_manually) {
                mmap_tags_import = mmap_preparation;
            }
            if (not is_mmap_index_build_manually) {
                mmap_index_build = mmap_preparation;
            }
            if (not is_mmap_osm_vertices_manually) {
                mmap_osm_vertices = mmap_preparation;
            }
            if (not is_mmap_osm_tags_manually) {
                mmap_osm_tags = mmap_preparation;
            }
        }
    }

    void tracks_data::correct(const po::variables_map &vm) {
        if (not vm["read-line"].empty() and not vm["read-line"].as<bool>()) {
            if (files.empty()) {
                throw std::invalid_argument{"no tracks file specified, please see help."};
            }

            if (vm["tracks-file-type"].empty() and vm["match-file-type"].empty() and vm["compare-file-type"].empty()) {
                std::string _file_extension{};
                for (const auto &file : files) {
                    file_extension = std::filesystem::path(file).extension().string();
                    if (_file_extension.empty()) {
                        _file_extension = file_extension;
                    }
                    if (file_extension != _file_extension) {
                        throw std::runtime_error{
                                "track files have inconsistent file extensions, please fix or override manually."
                        };
                    }
                }

                if (file_extension.starts_with('.')) {
                    file_extension = file_extension.substr(1);
                }
            }

            boost::to_lower(file_extension);

            if (file_extension != FILE_TYPE_CSV and file_extension != FILE_TYPE_GPX and
                file_extension != FILE_TYPE_LIST and file_extension != FILE_TYPE_SEATTLE) {
                throw std::invalid_argument{
                        std::format("track file extension {} unsupported, please see help.", file_extension)
                };
            }
        }
    }

    void matching_data::correct(const po::variables_map &vm) {
        if (candidate_search != CANDIDATE_SEARCH_CIRCLE and candidate_search != CANDIDATE_SEARCH_NEAREST and
            candidate_search != CANDIDATE_SEARCH_COMBINED and candidate_search != CANDIDATE_SEARCH_NEXT) {
            throw std::invalid_argument{std::format("candidate search type {} is unknown.", candidate_search)};
        }

        boost::trim(filter_polygon);
        boost::to_upper(filter_polygon);

        boost::trim(filter_method);

        if (filter_method != FILTER_METHOD_WITHIN and filter_method != FILTER_METHOD_INTERSECTS) {
            throw std::invalid_argument{std::format("filter method {} is invalid.", filter_method)};
        }

        const bool is_trajectory_simplification_manually = not vm["trajectory-simplification"].defaulted();
        const bool is_filter_duplicates_manually = not vm["filter-duplicates"].defaulted();
        const bool is_simplify_track_manually = not vm["simplify-track"].defaulted();
        const bool is_median_merge_manually = not vm["median-merge"].defaulted();
        const bool is_adaptive_median_merge_manually = not vm["adaptive-median-merge"].defaulted();
        const bool is_candidate_adoption_manually = not vm["candidate-adoption"].defaulted();
        const bool is_candidate_adoption_siblings_manually = not vm["candidate-adoption-siblings"].defaulted();
        const bool is_candidate_adoption_nearby_manually = not vm["candidate-adoption-nearby"].defaulted();
        const bool is_candidate_adoption_reverse_manually = not vm["candidate-adoption-reverse"].defaulted();
        const bool is_within_edge_turns_manually = not vm["within-edge-turns"].defaulted();

        if (is_trajectory_simplification_manually) {
            if (not is_filter_duplicates_manually) {
                filter_duplicates = trajectory_simplification;
            }
            if (not is_simplify_track_manually) {
                simplify_track = trajectory_simplification;
            }
            if (not is_median_merge_manually) {
                median_merge = trajectory_simplification;
            }
            if (not is_adaptive_median_merge_manually) {
                adaptive_median_merge = trajectory_simplification;
            }
        }

        if (is_candidate_adoption_manually) {
            if (not is_candidate_adoption_siblings_manually) {
                candidate_adoption_siblings = candidate_adoption;
            }
            if (not is_candidate_adoption_nearby_manually) {
                candidate_adoption_nearby = candidate_adoption;
            }
            if (not is_candidate_adoption_reverse_manually) {
                candidate_adoption_reverse = candidate_adoption;
            }
        }

        if (not candidate_adoption_nearby and not candidate_adoption_siblings and not candidate_adoption_reverse and
            not is_within_edge_turns_manually) {
            within_edge_turns = true;
        }

        if (radius > radius_upper_limit) {
            radius_upper_limit = radius;
        }
        if (radius < radius_lower_limit) {
            radius_lower_limit = radius;
        }
        if (candidate_search == CANDIDATE_SEARCH_NEXT) {
            k_nearest = 1;
        }

        if (not vm["quiet"].empty() and not vm["quiet"].as<bool>() and
            not vm["console"].empty() and not vm["console"].as<bool>()) {
            if (split_time > 0) {
                std::cout << std::format("Split tracks at {:.3f} seconds.", split_time) << std::endl;
            }
            if (not filter_polygon.empty()) {
                std::cout << "Filter tracks with method: " << filter_method << std::endl;
            }
            if (not filter_duplicates) {
                std::cout << "Disabled duplicate points filtering." << std::endl;
            }
            if (not simplify_track) {
                std::cout << "Disabled Douglas-Peucker track simplification." << std::endl;
            }
            if (not median_merge) {
                std::cout << "Disabled median-merge track simplification." << std::endl;
            }
            if (not adaptive_median_merge) {
                std::cout << "Disabled adaptive median-merge." << std::endl;
            }
            if (within_edge_turns) {
                std::cout << "Enabled within-edge-turn mode." << std::endl;
            }
            if (not candidate_adoption_siblings) {
                std::cout << "Disabled candidate adoption of siblings." << std::endl;
            }
            if (not candidate_adoption_nearby) {
                std::cout << "Disabled candidate adoption of nearby candidates." << std::endl;
            }
            if (candidate_adoption_reverse) {
                std::cout << "Enabled candidate adoption of reverse candidates." << std::endl;
            }
            if (not adaptive_radius) {
                std::cout << "Disabled adaptive radius." << std::endl;
            }
        }
    }

    void model_data::correct(const po::variables_map &vm) {
        if (model != MODEL_POLICY_ITERATION and model != MODEL_VALUE_ITERATION and
            model != MODEL_Q_LEARNING and model != MODEL_VITERBI) {
            throw std::invalid_argument{std::format("model type {} is unknown.", model)};
        }

        if (state_size < 2 or state_size > 3) {
            throw std::invalid_argument{"state-size must be 2 or 3."};
        }
    }

    void mode_data::correct(const po::variables_map &vm) {
        all.correct(vm);
        global.correct(vm);
    }

    void prepare_data::correct(const po::variables_map &vm) {
        mode_data::correct(vm);

        network.correct(vm);
        osm_import.correct(vm);
        srs.correct(vm);
        files.correct(vm);
        expo.correct(vm);
        process.correct(vm);
        memory.correct(vm);
    }

    void work_data::correct(const po::variables_map &vm) {
        mode_data::correct(vm);

        console.correct(vm);
        performance.correct(vm);
    }

    void match_data::correct(const po::variables_map &vm) {
        work_data::correct(vm);

        input.correct(vm);
        srs.correct(vm);
        files.correct(vm);
        memory.correct(vm);
        match_output.correct(vm);
        tracks.correct(vm);
        srs_tracks.correct(vm);
        csv.correct(vm);
        expo.correct(vm);
        matching.correct(vm);
        model.correct(vm);
        weights_mdp.correct(vm);
        weights_hmm.correct(vm);
        dynamic_programming.correct(vm);
        learning.correct(vm);
    }

    void compare_data::correct(const po::variables_map &vm) {
        work_data::correct(vm);

        compare_output.correct(vm);
        match.correct(vm);
        srs_match.correct(vm);
        csv_match.correct(vm);
        comparison.correct(vm);
        srs_compare.correct(vm);
        csv_compare.correct(vm);
    }

    po::options_description options_all(all_data &data) {
        po::options_description options("Available Options", global.console.cols);

        options.add_options()
                ("help",
                        "show this help, can be combined with an application mode for more help")
                ("version",
                        "show the version of this software")
                ("config,c", po::value<std::string>(&data.config)->default_value("options.conf"),
                        "configuration file, can contain settings for all options;\n"
                        "explicitly specified command line options override configuration file settings;\n"
                        "if explicitly specified, file needs to exist")
                ("verbose", po::bool_switch(&data.verbose),
                        "show more output during operation")
                ("quiet", po::bool_switch(&data.quiet),
                        "silent mode, deactivates any output, overrides verbose");

        return options;
    }

    po::options_description options_global(global_data &data, const MODE mode) {
        po::options_description options("Application Modes", global.console.cols);

        const std::string desc_prepare{
                "prepare mode for importing and converting network data into the application specific network format; "
                "usually, this mode needs to be run first at once, "
                "since all other modes need the resulting binary files from this mode."
        };
        const std::string desc_match{
                "map matching mode for command line operation for fitting GNSS tracks to the prepared road network."
        };
        const std::string desc_server{
                "map matching server mode for running the map matching interface as a HTTP web service.\n"
                "(NOT YET IMPLEMENTED)"
        };
        const std::string desc_compare{
                "compare mode for computing the accuracy, similarities, and differences "
                "from two different map matching runs;\n"
                "a typical use case is evaluating map matching results against given ground truth routes."
        };

        if (mode == MODE::PREPARE) {
            options.add_options()
                    ("prepare", po::bool_switch(&data.prepare)->required(), desc_prepare.c_str());
        } else if (mode == MODE::MATCH) {
            options.add_options()
                    ("match", po::bool_switch(&data.match)->required(), desc_match.c_str());
        } else if (mode == MODE::SERVER) {
            options.add_options()
                    ("server", po::bool_switch(&data.server)->required(), desc_server.c_str());
        } else if (mode == MODE::COMPARE) {
            options.add_options()
                    ("compare", po::bool_switch(&data.compare)->required(), desc_compare.c_str());
        } else {
            options.add_options()
                    ("prepare", po::bool_switch(&data.prepare), desc_prepare.c_str())
                    ("match", po::bool_switch(&data.match), desc_match.c_str())
                    ("server", po::bool_switch(&data.server), desc_server.c_str())
                    ("compare", po::bool_switch(&data.compare), desc_compare.c_str());
        }

        return options;
    }

    po::options_description options_input(input_data &data) {
        po::options_description options("Input Options", global.console.cols);

        options.add_options()
                ("input,i", po::value<std::string>(&data.input)->required(),
                        "input folder with the prepared network data binary files");

        return options;
    }

    po::options_description options_output(output_data &data, const std::string &default_filename) {
        po::options_description options("Output Options", global.console.cols);

        options.add_options()
                ("output,o", po::value<std::string>(&data.output),
                        "output folder for the application results")
                ("filename,f", po::value<std::string>(&data.filename)->default_value(default_filename),
                        "output filename in the output folder for the application results");

        return options;
    }

    po::options_description options_match_output(match_output_data &data) {
        auto output = options_output(data, "matches.csv");

        po::options_description options("Match Output Options", global.console.cols);

        options.add_options()
                ("columns", po::value<std::string>(&data.columns)->default_value(matching::DEFAULT_COLUMNS),
                        "output columns in order for the application results, separated by comma;\n"
                        "  \tid: id of the track, aggregated in case of multiple ids were specified\n"
                        "  \taborted: 'no' if track was successfully matched, 'yes' if max-time was reached before finishing\n"
                        "  \tduration: matching duration in seconds\n"
                        "  \ttrack: original track as it was imported as WKT\n"
                        "  \tprepared: prepared track as WKT after track sanitation, i.e., remove duplicates, median-merge\n"
                        "  \tmatch: map matching result as WKT\n"
                        "  \tstart_time: start time of track, i.e., first point time\n"
                        "  \tend_time: end time of track, i.e., last point time\n"
                        "  \ttrack_length: track length in meter\n"
                        "  \tprepared_length: prepared track length in meter\n"
                        "  \tmatch_length: match length in meter\n"
                        "  \ttrack_times: list of track point times (off by default)\n"
                        "  \tprepared_times: list of prepared track point times (off by default)\n"
                        "  \tedge_ids: list of original edge IDs (off by default; if used, combine with simplify osm-aware")
                ("export-timestamps", po::value<bool>(&data.export_timestamps)->default_value(false, "off"),
                        "export unix timestamps for start_time, end_time, track_times, and prepared_times; "
                        "by default is off and uses a human readable time format with the UTC time zone, see below")
                ("export-time-zone",
                        po::value<std::string>(&data.export_time_zone)->default_value(util::current_time_zone()),
                        "time-zone to use for time export; if empty, uses UTC; by default uses system time zone")
                ("export-edges", po::value<bool>(&data.export_edges)->default_value(false, "off"),
                        "export matched routes based on the complete network edges instead of the actual parts, "
                        "this is only useful if the routes are later compared to matches that come from a list of edge IDs, "
                        "it returns routes with parts in the beginning and ending of the track, and at turns, "
                        "that have no equivalent part in the track, "
                        "however, the resulting routes overlap the network edges perfectly "
                        "since they do not start, end, and turn within the edges at new points matching the track")
                ("join-merges", po::value<bool>(&data.join_merges)->default_value(true, "on"),
                        "by default, the routes between candidates are concatenated, "
                        "this parameter removes additional joint points when possible, "
                        "so all joint points are removed except when reverse movements within an edge happen");

        output.add(options);

        return output;
    }

    po::options_description options_compare_output(compare_output_data &data) {
        auto output = options_output(data, "comparison.csv");

        po::options_description options("Comparison Output Options", global.console.cols);

        options.add_options()
                ("columns", po::value<std::string>(&data.columns)->default_value(compare::DEFAULT_COLUMNS),
                        "output columns in order for the application results, separated by comma;\n"
                        "  \tid: id of the track, aggregated in case of multiple ids were specified\n"
                        "  \tduration: matching duration in seconds\n"
                        "  \tmatch: map matching result as WKT\n"
                        "  \tground_truth: ground truth route as WKT\n"
                        "  \tmatch_length: match length in meter\n"
                        "  \tground_truth_length: ground truth length in meter\n"
                        "  \tcorrect_fraction: fraction of correctly matched line parts as percentage\n"
                        "  \terror_fraction: error fraction after formula from Newson-Krumm paper\n"
                        "  \tcorrect: correct matching length in meter\n"
                        "  \terror_added: erroneously added match parts length not present in ground truth in meter\n"
                        "  \terror_missed: erroneously missed ground truth parts length not present in match in meter\n"
                        "  \tcorrects: correct overlapping segments as WKT\n"
                        "  \terror_adds: erroneously added segments from match not present in ground truth as WKT\n"
                        "  \terror_misses: erroneously missed segments from ground truth not present in mach as WKT");

        output.add(options);

        return output;
    }

    po::options_description options_console(console_data &data) {
        po::options_description options("Console Options", global.console.cols);

        options.add_options()
                ("read-line", po::bool_switch(&data.read_line),
                        "read-line mode, read WKT track from console input, overrides track input files")
                ("console", po::bool_switch(&data.console),
                        "console output mode, write WKT match result to console, overrides output files and enables quiet mode for log messages");

        return options;
    }

    po::options_description options_prepare_network(prepare_network_data &data) {
        po::options_description options("Prepare Network Options", global.console.cols);

        options.add_options()
                ("file,i", po::value<std::vector<std::string>>(&data.files)->required(),
                        "paths to input network files, can be specified multiple times for multiple files;\n"
                        "for OSM files, imports all files one after another, strips duplicate entries,\n"
                        "and tries to combine the networks, if possible, else contains them as individual isles;\n"
                        "in arc-node file format, first file is the arcs file, second file is the nodes file,\n"
                        "can also be specified in pairs multiple times for importing all arc-node files at once\n"
                        "into a large network, duplicates are removed by spatial location")
                ("output,o", po::value<std::string>(&data.output)->required(),
                        "output folder for the prepared network data in binary files and the configuration")
                ("format", po::value<std::string>(&data.format)->default_value("osm"),
                        "input network format, see the following options:\n"
                        "\"osm\"\n"
                        "  \tOpenStreetMap (OSM) file format, for example for .osm.pbf, .osm.bz2, and others\n"
                        "  \tmultiple OSM files via --file can be imported at once, if they overlap, use --remove-duplicate-edges.\n"
                        "\"arc-node\"\n"
                        "  \tarc-node file format for the map matching dataset from Kubicka, M. et al. (2016)\n"
                        "  \t\"Dataset for testing and training map-matching methods\"\n"
                        "  \tfirst --file needs to be the nodes file, second --file the arcs file, because nodes are imported in advance of arcs\n"
                        "  \tthis pair-wise declaration can be repeated multiple times for importing all arc-node files at once.\n"
                        "\"seattle\"\n"
                        "  \tseattle file format for the ground truth dataset from Newson and Krumm (2009)\n"
                        "  \t\"Hidden Markov Map Matching through Noise and Sparseness\"\n"
                        "  \tmultiple --file can be imported in case it is needed.\n"
                        "  \tautomatically enables --remove-duplicate-nodes due to the original data containing such.\n"
                        "\"melbourne\"\n"
                        "  \tmelbourne file format for the ground truth dataset from Li, Hengfeng (2014)\n"
                        "  \t\"Spatio-temporal trajectory simplification for inferring travel paths\"\n"
                        "  \tfirst --file needs to be the vertex file, second --file the edges file, third --file the streets file\n"
                        "  \tthis triple-wise declaration can be repeated multiple times in case it is needed.\n"
                        "\"giscup\"\n"
                        "  \tgiscup file format for the map matching dataset from Ali, M. et al. (2012)\n"
                        "  \t\"ACM SIGSPATIAL GIS Cup 2012\"\n"
                        "  \tfirst --file needs to be the nodes file, second --file the edges file, third --file the edge-geometry file\n"
                        "  \tthis triple-wise declaration can be repeated multiple times in case it is needed.\n"
                        "default is \"osm\"");

        return options;
    }

    po::options_description options_prepare_osm_import(prepare_osm_import_data &data) {
        po::options_description options("Prepare OSM Import Options", global.console.cols);

        options.add_options()
                ("osm-profile", po::value<std::string>(&data.profile)->default_value("car"),
                        "profile for osm import, default is car road network, can be set to:\n"
                        "  \t\"car\": car road network profile\n"
                        "  \t\"manual\": manual profile with tags explicitely specified")
                ("osm-query", po::value<std::string>(&data.query)->default_value("true"),
                        "osm tag specification string for ways that are kept when they match the specified filter\n"
                        "filter specification in Backus-Naur Form (BNF):\n\n"
                        "<SPEC> ::= <DEFAULT> { \"|\" <RULE> }*\n"
                        "<DEFAULT> ::= \"true\" | \"false\"\n"
                        "<RULE> ::= <DEFAULT> \",\" <TAG> [ \"=\" <VALUE> ]\n"
                        "<TAG> ::= <string>\n"
                        "<VALUE> ::= <string>\n\n"
                        "Examples:\n"
                        "  \t\"true\"\n"
                        "  \taccepts all tags\n"
                        "  \t\"false|true,highway\"\n"
                        "  \taccepts only highway tags\n"
                        "  \t\"false|false,highway=construction|true,highway\"\n"
                        "  \taccepts only highways that are not construction (the order is important)")
                ("osm-filter", po::value<std::string>(&data.filter)->default_value("true"),
                        "osm tag specification string for ways that are filtered after they were queried, "
                        "so this removes ways in a second pass that were previously loaded\n"
                        "see osm-query for filter specification\n"
                        "Example:\n"
                        "  \t\"true|false,access=no|false,access=private\"\n"
                        "  \taccept all tags except ways that are not accessible or private")
                ("osm-oneway", po::value<std::string>(&data.oneway)->default_value("false"),
                        "osm tag specification string for oneway description, "
                        "a queried and not filtered way is set to a directed oneway edge in the graph if it matches this filter, "
                        "all non-oneway ways become two-way edges, i.e., two directed edges in both directions in the graph\n"
                        "see osm-query for filter specification\n"
                        "Example:\n"
                        "  \t\"false|true,oneway=yes|true,oneway=true\"\n"
                        "  \tall ways tagged with oneway=yes or oneway=true become directed edges")
                ("osm-oneway-reverse", po::value<std::string>(&data.oneway_reverse)->default_value("false"),
                        "osm tag specification string for oneway reverse description, "
                        "some oneways are tagged reversed, so if this filter matches, only the opposing directed edge is kept "
                        "in the same manner as if it had a normal oneway tag initially\n"
                        "see osm-query for filter specification\n"
                        "Example:\n"
                        "  \t\"false|true,oneway=reverse\"\n"
                        "  \tall ways tagged with oneway=reverse become directed edges in the opposing direction");

        return options;
    }

    po::options_description options_srs(srs_data &data, const std::string &prefix) {
        po::options_description options("Spatial Reference System Options", global.console.cols);

        options.add_options()
                ((prefix + "srs").c_str(), po::value<std::int32_t>(&data._srs)->default_value(4326),
                        "spatial reference system (srs) as EPSG code, default is WGS84 (EPSG 4326)")
                ((prefix + "srs-type").c_str(), po::value<std::string>(&data._srs_type),
                        "srs type, can be set to 'geographic', 'spherical-equatorial', or 'cartesian';\n"
                        "by default automatically detects 'geographic' for srs with latitude and longitude\n"
                        "and 'cartesian' for srs with x and y coordinates;\n"
                        "'spherical-equatorial' can be set for coordinates in latitude and longitude for faster\n"
                        "but less accurate length and angle calculations (using haversine) than 'geographic'");

        return options;
    }

    po::options_description options_srs_transform(srs_transform_data &data, const std::string &prefix) {
        auto srs = options_srs(data, prefix);

        po::options_description options("Spatial Reference System Reprojection Options", global.console.cols);

        options.add_options()
                ((prefix + "transform-srs").c_str(), po::value<std::int32_t>(&data._transform_srs),
                        "transform spatial reference system (srs) as EPSG code;\n"
                        "when defined, data is transformed to given srs;\n"
                        "use EPSG code 3857 as recommendation for cartesian WGS84 Pseudo-Mercator projected coordinate system")
                ((prefix + "transform-srs-type").c_str(), po::value<std::string>(&data._transform_srs_type),
                        "transform srs type, can be set to 'geographic', 'spherical-equatorial', or 'cartesian';\n"
                        "by default automatically detects type, see '--srs-type' for details");

        srs.add(options);

        return srs;
    }

    po::options_description options_prepare_srs(prepare_srs_data &data) {
        po::options_description options("Prepare Spatial Reference System Options", global.console.cols);

        options.add(options_srs_transform(data, ""));

        return options;
    }

    po::options_description options_files(files_data &data) {
        po::options_description options("Files Options", global.console.cols);

        options.add_options()
                ("graph-file", po::value<std::string>(&data.graph_file)->default_value("graph.bin"),
                        "memory mapped or binary persistent graph file")
                ("tags-file", po::value<std::string>(&data.tags_file)->default_value("tags.bin"),
                        "memory mapped or binary persistent tags file")
                ("index-file", po::value<std::string>(&data.index_file)->default_value("index.bin"),
                        "memory mapped or binary persistent index file");

        return options;
    }

    po::options_description options_prepare_files(prepare_files_data &data) {
        po::options_description options("Prepare Files Options", global.console.cols);

        options.add_options()
                ("graph-file", po::value<std::string>(&data.graph_file)->default_value("graph.bin"),
                        "memory mapped or binary persistent file for graph storage;\n"
                        "used for storing the finalized compressed sparse row road network after modifications are made")
                ("tags-file", po::value<std::string>(&data.tags_file)->default_value("tags.bin"),
                        "memory mapped or binary persistent file for tags storage;\n"
                        "used for storing the final tags that are actually referenced from the final graph road network")
                ("index-file", po::value<std::string>(&data.index_file)->default_value("index.bin"),
                        "memory mapped or binary persistent file for spatial index storage;\n"
                        "used for storing the spatial points and segments indices for the graph road network")
                ("graph-import-file", po::value<std::string>(&data.graph_import_file)->default_value("tmp_graph.bin"),
                        "memory mapped temporary file for graph storage during import;\n"
                        "used for storing the mutable adjacency list road network, needed for making modifications to the graph, "
                        "(e.g., simplify, remove-unconnected), automatically deleted when no longer needed;\n"
                        "is only used when memory-mapped-graph-import is on")
                ("tags-import-file", po::value<std::string>(&data.tags_import_file)->default_value("tmp_tags.bin"),
                        "memory mapped temporary file for osm tags storage during graph import;\n"
                        "used for storing the osm tags that are actually imported from the osm data, depending on the tag filters used,\n"
                        "automatically deleted when no longer needed;\n"
                        "is only used when memory-mapped-tags-import is on")
                ("index-build-file",
                        po::value<std::string>(&data.index_build_file)->default_value("tmp_index_build.bin"),
                        "memory mapped temporary file for spatial index build storage;\n"
                        "used for storing the spatial points and segments data extracted from the final graph "
                        "that are then used to pack-build the spatial indices, automatically deleted when no longer needed;\n"
                        "is only used when memory-mapped-index-build is on")
                ("osm-vertices-file",
                        po::value<std::string>(&data.osm_vertices_file)->default_value("tmp_osm_vertices.bin"),
                        "memory mapped temporary file for osm vertices storage;\n"
                        "used for storing the point geometries that the osm ways reference to, "
                        "automatically deleted when no longer needed;\n"
                        "is only used when memory-mapped-osm-vertices is on")
                ("osm-tags-file", po::value<std::string>(&data.osm_tags_file)->default_value("tmp_osm_tags.bin"),
                        "memory mapped temporary file for osm tags storage;\n"
                        "used for storing the osm tags that are seen during osm import, "
                        "which are later used as reference for the graph, automatically deleted when no longer needed;\n"
                        "is only used when memory-mapped-osm-tags is on")
                ("initial-file-size",
                        po::value<std::size_t>(&data.initial_size)->default_value(128 * 1024 * 1024),
                        "initial file size in bytes for sparse memory mapped files;\n"
                        "the higher the value the less often a file needs to be flushed, closed, grown, and reopened during import,\n"
                        "the sparse files do not actually consume the whole space, only the used part is actually consumed,\n"
                        "the files are deleted if no longer needed or shrunken to the actual size after the import is finished;\n"
                        "default value is 128 MiB = 128 * 1024 * 1024 bytes;\n"
                        "in some memory-mapping configurations, the initial-file-size is automatically set to the disk limit,\n"
                        "because some data structures cannot be resized and need a large enough target space in advance,\n"
                        "therefore, memory-mapping should be used with caution on filesystems that don't support sparse files");

        return options;
    }

    po::options_description options_export(export_data &data) {
        po::options_description options("Export Options", global.console.cols);

        options.add_options()
                ("candidates", po::value<std::string>(&data.candidates),
                        "export candidates to the given filename, if no filename is given, no candidates are exported;\n"
                        "filename is appended to output folder");
        options.add_options()
                ("candidates-columns",
                        po::value<std::string>(&data.candidates_columns)->default_value(
                                matching::DEFAULT_CANDIDATE_COLUMNS),
                        "output columns in order for the candidate exports, separated by comma;\n"
                        "  \tid: id of the track, aggregated in case of multiple ids were specified\n"
                        "  \tpart_index: index of the track part (i.e., for multi-line tracks)\n"
                        "  \tset_index: index of the candidate set\n"
                        "  \tcandidate_index: index of the candidate within a candidate set\n"
                        "  \ttime: track point time\n"
                        "  \tprojection: candidate projection from track point to road network point\n"
                        "  \tdistance: distance in meter\n"
                        "  \tfrom: route from start of the road network edge to the candidate point as WKT\n"
                        "  \tto: route from the candidate point to the end of the road network edge as WKT\n"
                        "  \tedge_id: original edge ID the candidate lies on\n"
                        "  \tis_result: true if this candidate is part of the resulting match");

        return options;
    }

    po::options_description options_prepare_export(prepare_export_data &data) {
        po::options_description options("Prepare Export Options", global.console.cols);

        options.add_options()
                ("export-nodes-csv-file", po::value<std::string>(&data.export_nodes_csv_file),
                        "filename for exporting the finalized network nodes in CSV-format;\n"
                        "the file (for example \"nodes.csv\") is created in the output folder.")
                ("export-edges-csv-file", po::value<std::string>(&data.export_edges_csv_file),
                        "filename for exporting the finalized network edges in CSV-format;\n"
                        "the file (for example \"edges.csv\") is created in the output folder.")
                ("export-network-osm-file", po::value<std::string>(&data.export_network_osm_file),
                        "export final network to .osm.pbf file format;\n"
                        "might contain the data in different order than the import file;\n"
                        "in case of third-party input networks without tags, the ways are given the highway tag");

        return options;
    }

    po::options_description options_prepare_process(prepare_process_data &data) {
        po::options_description options("Prepare Process Options", global.console.cols);

        options.add_options()
                ("simplify", po::value<std::string>(&data.simplify)->default_value("all"),
                        "simplify network graph, highly recommended for improving routing and matching speed drastically;\n"
                        "initially the osm data needs to be imported with graph nodes for every straight segment,\n"
                        "simplify merges adjacent straight segments to lines and remains only the graph nodes needed for routing,\n"
                        "for example at junctions; the spatial information (for example curves) of the graph is not altered;\n"
                        "the following options can be used:\n"
                        "\"all\"\n"
                        "  \tsimplify as much as possible, even combine roads with different osm ids and tags, recommended\n"
                        "\"none\"\n"
                        "  \tdo not simplify, road network remains bigger and slower, but might be needed to improve accuracy\n"
                        "  \tin special cases, e.g., when comparing the results to ground-truth that is based on the original edges\n"
                        "\"osm-aware\"\n"
                        "  \tsimplify, but don't combine roads with different osm ids and tags,\n"
                        "  \tthis mode is usually not recommended, except special .csv extracts are of interest\n"
                        "  \tor when the map matching metrics later down the line respect the osm tags,\n"
                        "  \twhich they do not by design currently;\n"
                        "  \thowever, use this when you export edge_ids with the matches later,\n"
                        "  \tas only then the ids are correctly preserved during simplification\n"
                        "default is \"all\"")
                ("remove-unconnected", po::value<bool>(&data.remove_unconnected)->default_value(false, "off"),
                        "remove weakly unconnected components so that only the largest subgraph remains;\n"
                        "when multiple isle networks are imported, only the largest isle remains;\n"
                        "usually is only interesting for removing tiny network isles in large networks")
                ("remove-duplicate-nodes", po::value<bool>(&data.remove_duplicate_nodes)->default_value(false, "off"),
                        "remove duplicate nodes by comparing exact coordinates;\n"
                        "usually is needed if networks were split in multiple parts with overlapping nodes on the outer border")
                ("remove-duplicate-edges", po::value<bool>(&data.remove_duplicate_edges)->default_value(false, "off"),
                        "remove duplicate edges by comparing id, geometric line, and tags;\n"
                        "usually is needed if multiple networks with overlapping parts are imported at once");

        return options;
    }

    po::options_description options_memory(memory_data &data) {
        po::options_description options("Memory Options", global.console.cols);

        options.add_options()
                ("memory-mapping", po::value<bool>(&data.memory_mapping)->default_value(true, "on"),
                        "this tool uses memory-mapped files for graph, index, and OSM import storage by default, "
                        "by disabling memory-mapping, the data can be processed completely in the main memory (RAM), "
                        "in this case binary files consisting of memory dumps of the graph and indices are used, "
                        "during the memory mapped preparation, some files are only temporary and discarded after;\n"
                        "the later steps of this tool suite need to have the same memory-mode configuration\n"
                        "for example, non-memory-mapped graph and indices files cannot be used in memory-mapped mode;\n"
                        "starting up from a memory dump is slower than starting up from memory-mapped files, "
                        "but processing speed is faster with in-memory configuration compared to memory-mapped files;\n"
                        "we recommend using memory-mapped files, especially when the underlying storage is of SSD or NVMe type, "
                        "because the main memory usage grows very large when using in-memory configuration")
                ("memory-mapped-graph", po::value<bool>(&data.mmap_graph)->default_value(true, "on"),
                        "option for disabling memory mapping for the final graph;\n"
                        "when specified overrides global memory-mapping configuration")
                ("memory-mapped-indices", po::value<bool>(&data.mmap_indices)->default_value(true, "on"),
                        "option for disabling memory mapping for the final indices;\n"
                        "when specified overrides global memory-mapping configuration")
                ("memory-mapped-tags", po::value<bool>(&data.mmap_tags)->default_value(true, "on"),
                        "option for disabling memory mapping for the final tags;\n"
                        "when specified overrides global memory-mapping configuration");

        return options;
    }

    po::options_description options_prepare_memory(prepare_memory_data &data) {
        auto memory = options_memory(data);

        po::options_description options("Prepare Memory Options", global.console.cols);

        options.add_options()
                ("memory-mapped-preparation", po::value<bool>(&data.mmap_preparation)->default_value(true, "on"),
                        "option for disabling memory mapping for the intermediate temporary preparation files;\n"
                        "the final memory mapped files needed for the map matching software will be created, "
                        "but the intermediate temporary ones, which would automatically be deleted after import, are not used;\n"
                        "this drastically increases the importing speed and drastically reduces the disk writes, "
                        "also the resulting files are the same, but a lot more main memory is needed during preparation "
                        "compared to this option disabled, so use only wich enough main memory;\n"
                        "when specified overrides global memory-mapping configuration")
                ("memory-mapped-graph-import", po::value<bool>(&data.mmap_graph_import)->default_value(true, "on"),
                        "option for disabling memory mapping for the temporary graph import file;\n"
                        "when specified overrides global memory-mapping-preparation configuration")
                ("memory-mapped-tags-import", po::value<bool>(&data.mmap_tags_import)->default_value(true, "on"),
                        "option for disabling memory mapping for the temporary import tags file;\n"
                        "when specified overrides global memory-mapping-preparation configuration")
                ("memory-mapped-index-build", po::value<bool>(&data.mmap_index_build)->default_value(true, "on"),
                        "option for disabling memory mapping for the temporary index build file;\n"
                        "when specified overrides global memory-mapping-preparation configuration")
                ("memory-mapped-osm-vertices", po::value<bool>(&data.mmap_osm_vertices)->default_value(true, "on"),
                        "option for disabling memory mapping for the temporary OSM vertices file;\n"
                        "when specified overrides global memory-mapping-preparation configuration")
                ("memory-mapped-osm-tags", po::value<bool>(&data.mmap_osm_tags)->default_value(true, "on"),
                        "option for disabling memory mapping for the temporary OSM tags file;\n"
                        "when specified overrides global memory-mapping-preparation configuration");

        memory.add(options);

        return memory;
    }

    po::options_description options_performance(performance_data &data) {
        po::options_description options("Performance Options", global.console.cols);

        options.add_options()
                ("threads", po::value<std::uint32_t>(&data.threads)->default_value(std::thread::hardware_concurrency()),
                        "threads that are used for parallel (i.e., multi-threading) matching or comparing;\n"
                        "set to 1 for disabling multi-threading, by default uses the available system threads");

        return options;
    }

    po::options_description options_tracks(tracks_data &data, const std::string &prefix) {
        po::options_description options("Tracks Options", global.console.cols);

        options.add_options()
                ((prefix + "tracks").c_str(), po::value<std::vector<std::string>>(&data.files),
                        "path to track(s) file;\n"
                        "can be specified multiple times for multiple track files")
                ((prefix + "tracks-file-type").c_str(), po::value<std::string>(&data.file_extension),
                        "track file type;\n"
                        "by default is automatically detected from (the first) filename extension;\n"
                        "can be manually overridden to the following supported file types:\n"
                        "  \t\"csv\"\n"
                        "  \t\"gpx\"\n"
                        "  \t\"list\" (only for matching)\n"
                        "  \t\"seattle\" (only for matching)\n"
                        "the \"list\" and \"seattle\" file types are special extensions, "
                        "they are meant to be specified manually with specific input files, "
                        "and they only work in matching mode, not in comparison mode;\n"
                        "the \"list\" type awaits as input a list of network edge IDs, "
                        "it then exports the track as WKT, based on the given network, "
                        "used for route files from Li, Hengfeng (2014), Kubicka, M. et al. (2016), and Ali, M. et al. (2012), "
                        "automatically resolves gaps, duplicate node ids, and duplicate edge ids, "
                        "if there are multiple columns, use --delimiter and --id with a number for the edge ID column, "
                        "--skip-lines is also supported for skipping headers as they are not needed and unsupported\n"
                        "the \"seattle\" type is for exporting the ground truth route as WKT track "
                        "from the seattle dataset from Newson and Krumm (2009), "
                        "the first file is the road network file, the second file the ground truth file, "
                        "the additional option is because the \"seattle\" format needs a special handling for the direction");

        return options;
    }

    po::options_description options_csv(csv_data &data, const std::string &prefix) {
        po::options_description options("CSV Options", global.console.cols);

        options.add_options()
                ((prefix + "delimiter").c_str(), po::value<std::string>(&data.delimiter)->default_value(","),
                        "delimiter for .csv tracks file, only for import, "
                        "export always uses ',' (because csv means 'comma' separated);\n"
                        "for tabulator you can also use '\\t', and for space '\\s', "
                        "they are replaced with the true characters before being applied")
                ((prefix + "grouped").c_str(), po::value<bool>(&data.grouped)->default_value(true, "yes"),
                        "whether the .csv file is grouped by id, which is beneficial in coordinates read mode (non-WKT mode) "
                        "for faster parsing of tracks with reduced memory footprint; grouped means that the id column is sorted "
                        "in such a way that all rows with the same id are put together, not mixed or interrupted by rows with "
                        "different ids; in case grouped is disabled, the whole .csv file needs to be parsed and grouped by id "
                        "before the matching starts; this setting is ignored in WKT read mode")
                ((prefix + "skip-lines").c_str(), po::value<std::size_t>(&data.skip_lines)->default_value(0),
                        "skip specified number of lines at the head of the tracks .csv file, "
                        "works also in combination with no-header, "
                        "e.g., set to 1 with no-header to skip an existing header to use number indices for columns instead")
                ((prefix + "no-header").c_str(), po::bool_switch(&data.no_header),
                        "csv has no header, column specifications are interpreted as indices of columns")
                ((prefix + "no-id").c_str(), po::bool_switch(&data.no_id),
                        "csv has no id column, in this case, the id is an incrementing integer starting from 0, "
                        "see also --no-id-whole-file setting")
                ((prefix + "no-id-whole-file").c_str(), po::bool_switch(&data.no_id_whole_file),
                        "when active, the whole file, i.e., all rows get the same id, "
                        "when disabled (default), the id is incremented with each row of data; "
                        "use this setting with x,y coordinate points when --no-id is specified so that the whole file "
                        "becomes one track, i.e., combine all points or the whole file into one track")
                ((prefix + "id").c_str(), po::value<std::vector<std::string>>(&data.ids)->default_value({"id"}, "id"),
                        "id column(s) of tracks .csv file, "
                        "can be specified multiple times for composite primary keys over multiple columns, "
                        "internally, the id-aggregator field is then used for concatenation of the values")
                ((prefix + "id-aggregator").c_str(), po::value<std::string>(&data.id_aggregator)->default_value("_"),
                        "used for aggregating the values of composite primary keys, see id")
                ((prefix + "wkt").c_str(), po::bool_switch(&data.wkt),
                        "use WKT read mode for .csv tracks file instead of default x,y coordinates read mode")
                ((prefix + "x").c_str(), po::value<std::string>(&data.x)->default_value("x"),
                        "x column (or longitude) of tracks .csv file in coordinates read mode")
                ((prefix + "y").c_str(), po::value<std::string>(&data.y)->default_value("y"),
                        "y column (or latitude) of tracks .csv file in coordinates read mode")
                ((prefix + "geometry").c_str(), po::value<std::string>(&data.geometry)->default_value("geometry"),
                        "geometry column of tracks .csv file in WKT read mode "
                        "can either be of type POINT or LINESTRING, "
                        "in POINT mode, points are sorted ascending by time column, if available, else by appearance, "
                        "in LINESTRING mode, timestamp is just an incremented value for each point")
                ((prefix + "selector").c_str(), po::value<std::vector<std::string>>(&data.selectors),
                        "track selector for matching a specific track from the tracks file, can be specified multiple times");

        return options;
    }

    po::options_description options_csv_tracks(csv_tracks_data &data, const std::string &prefix) {
        auto csv = options_csv(data, prefix);

        po::options_description options("CSV/GPX Time Options", global.console.cols);

        options.add_options()
                ((prefix + "time").c_str(),
                        po::value<std::vector<std::string>>(&data.time)->default_value({"time"}, "time"),
                        "time column(s) of tracks .csv file, is optional if no such column exists, for example in WKT LINESTRING read mode, "
                        "currently, the time column(s) are only used for sorting points while reading tracks .csv file, is ignored for .gpx files; "
                        "can be specified multiple times similarly as --id, the time columns are first aggregated "
                        "by --time--aggregator and then parsed by the --time-format, "
                        "so the aggregator must be part of the format, if multiple time columns exist")
                ((prefix + "time-aggregator").c_str(),
                        po::value<std::string>(&data.time_aggregator)->default_value("_"),
                        "used for aggregating the values of composite time keys, see time")
                ((prefix + "time-format").c_str(), po::value<std::string>(&data.time_format)->default_value("%FT%T%Oz"),
                        "time format for parsing the time value in the time column into unix timestamp format, "
                        "please review std::chrono::parse documentation for syntax, "
                        "default reads ISO 8601 date and time with time zone")
                ((prefix + "no-parse-time").c_str(), po::bool_switch(&data.no_parse_time),
                        "do not parse time, for example when the time is already in unix timestamp format (seconds), "
                        "or when the time is not parsable at all (in this case, increments 1 per second starting from 0), "
                        "or with .gpx files with invalid time attribute");

        csv.add(options);

        return csv;
    }

    po::options_description options_matching(matching_data &data) {
        po::options_description options("Matching Options", global.console.cols);

        options.add_options()
                ("max-time", po::value<double>(&data.max_time)->default_value(0.0, "0.0"),
                        "when max-time (defined in seconds) is set to anything above 0.0, "
                        "the matching of a track is stopped after the set max-time is reached;"
                        "there are multiple checkpoints during the matching algorithm for a defined aborting, "
                        "this means that it can take a slight amount of extra time before the matching is actually stopped, "
                        "in case a matching was stopped because the max-time was reached, the 'aborted' column is set to 'yes' in the output file, "
                        "in case the current algorithm was able to produce any result up to this point, it is outputted, "
                        "else nothing is outputted; for example Q-Learning might be able to output something suboptimal")
                ("split-time", po::value<double>(&data.split_time)->default_value(0.0, "0"),
                        "time in seconds as time delta for splitting a track into multiple individual tracks; "
                        "when two succeeding points with a timestamp have a difference at least as large as this value, "
                        "the track is split into two parts, and so on, until all individual track parts have no time delta"
                        "between any two points that exceeds this value; setting to 0 disables this functionality")
                ("filter-polygon", po::value<std::string>(&data.filter_polygon)->default_value(""),
                        "filter definition of polygon or bounding box as WKT; "
                        "tracks that do not lie within or intersect with this filter are not matched, "
                        "see below for changing the method; "
                        "a polygon is defined counterclockwise with x and y coordinates, "
                        "the first and last coordinates have to be equal so that the polygon is closed: "
                        "POLYGON ((30 10, 40 40, 20 40, 10 20, 30 10));\n"
                        "a bounding box can also be defined as the minimum and maximum x and y coordinates: "
                        "BOX (10 20, 40 50);\n"
                        "for a bounding box from x coordinates 10 to 40 and y coordinates 20 to 50; "
                        "the filter should be defined in the same spatial reference system as the tracks")
                ("filter-method", po::value<std::string>(&data.filter_method)->default_value(FILTER_METHOD_WITHIN),
                        "filter method used with the filter polygon, possible options:"
                        "\"within\"\n"
                        "  \tonly keeps tracks that lie completely within the polygon with all points,\n"
                        "  \tinternally uses covered-by and keeps points on the border\n"
                        "\"intersects\"\n"
                        "  \tonly keeps tracks that lie within the polygon or intersect the polygon somewhere\n")
                ("trajectory-simplification",
                        po::value<bool>(&data.trajectory_simplification)->default_value(true, "on"),
                        "switch for disabling all steps of trajectory simplification, "
                        "disables 'filter-duplicates', 'simplify-track', 'median-merge', "
                        "and 'adaptive-median-merge' when set to off; enabled by default")
                ("filter-duplicates", po::value<bool>(&data.filter_duplicates)->default_value(true, "on"),
                        "filter duplicate (adjacent spatially equal) points in tracks before matching, is recommended")
                ("simplify-track", po::value<bool>(&data.simplify_track)->default_value(true, "on"),
                        "simplify track with Douglas-Peucker algorithm")
                ("simplify-track-distance-tolerance",
                        po::value<double>(&data.simplify_track_distance_tolerance)->default_value(5.0, "5.0"),
                        "distance tolerance in meters around line for Douglas-Peucker algorithm")
                ("median-merge", po::value<bool>(&data.median_merge)->default_value(true, "on"),
                        "simplify track by merging point clouds to their median point coordinates, "
                        "this reduces the amount of points when vehicles emit positions from static positions")
                ("median-merge-distance-tolerance",
                        po::value<double>(&data.median_merge_distance_tolerance)->default_value(10.0, "10.0"),
                        "distance tolerance in meters around a point to merge all points within the given radius")
                ("adaptive-median-merge", po::value<bool>(&data.adaptive_median_merge)->default_value(true, "on"),
                        "adapt median-merge-distance-tolerance automatically for point clouds that are far from streets, "
                        "the distance can be raised up to half the distance of the nearest road edge for merging")
                ("within-edge-turns",
                        po::value<bool>(&data.within_edge_turns)->default_value(false, "off"),
                        "enables turns within edges, "
                        "when disabled, turns are only possible at crossroads and junctions (nodes of the graph), "
                        "when enabled, turns within edges (roads of the graph) become possible, "
                        "this is done by trimming the path from the current position to the next node and back, "
                        "by default this is disabled, because it reduces correctness of results with candidate-adoption enabled "
                        "as large detours are needed for making the optimizer collapsing candidates, "
                        "it is, however, enabled by default when all candidate-adoption features are disabled, "
                        "because then it raises the correctness of the results as no larger detours are taken "
                        "that could not be collapsed without candidate-adoption")
                ("a-star", po::bool_switch(&data.a_star),
                        "use the A* shortest path algorithm instead of Dijkstra's shortest path algorithm, "
                        "may in fact decrease performance in most situations, because the A* algorithm is a "
                        "single-source-single-target algorithm and this software benefits from "
                        "single-source-multiple-target queries as Dijsktra's algorithm provides; "
                        "moreover the accuracy might be worse, because the A* algorithm might not yield optimal results "
                        "in geographic and spherical coordinate systems due to the heuristic method not being admissible")
                ("routing-max-distance-factor",
                        po::value<double>(&data.routing_max_distance_factor)->default_value(5.0, "5.0"),
                        "max distance factor for upper bound for routing algorithm, "
                        "removes all nodes from routing that are too far away from the search area between a start and end node, "
                        "dramatically reduces routing duration in networks significantly larger than the given track, "
                        "any negative value (i.e., -1) disables this setting")
                ("candidate-adoption", po::value<bool>(&data.candidate_adoption)->default_value(true, "on"),
                        "switch for disabling all steps of candidate adoption, "
                        "disables 'candidate-adoption-siblings', 'candidate-adoption-nearby', "
                        "and 'candidate-adoption-reverse' when set to off; enabled by default, "
                        "but does not activate 'candidate-adoption-reverse' when not explicitly enabled")
                ("candidate-adoption-siblings",
                        po::value<bool>(&data.candidate_adoption_siblings)->default_value(true, "on"),
                        "for each measurement, adopt candidates from preceding and succeeding candidate, "
                        "in other words, each measurement can choose from at least three road positions "
                        "and each road position is referenced by three candidates, "
                        "gives much better results but has a huge calculation speed impact")
                ("candidate-adoption-nearby",
                        po::value<bool>(&data.candidate_adoption_nearby)->default_value(true, "on"),
                        "for each measurement, adopt candidates from searching nearby candidates "
                        "intersecting a search circle with distance of the farthest current candidate, "
                        "in dense areas with very near records this exchanges all candidates between all records, "
                        "gives much better results in dense record areas but has a very huge calculation speed impact")
                ("candidate-adoption-reverse",
                        po::value<bool>(&data.candidate_adoption_reverse)->default_value(false, "off"),
                        "for each nearby adoption, inject current candidate into candidate sets of adopting candidates "
                        "this is especially useful when adaptive optimization rounds are enabled, "
                        "new found candidates are then deployed to preceding and following locations around the high error location, "
                        "has the highest calculation speed impact")
                ("candidate-search", po::value<std::string>(&data.candidate_search)->default_value("combined"),
                        "candidate search algorithm to use, possible options:\n"
                        "\"circle\"\n"
                        "  \tsearches for all edges around a point intersecting a circle with at least given radius\n"
                        "\"nearest\"\n"
                        "  \tsearches the k-nearest edges around a point\n"
                        "\"combined\"\n"
                        "  \tsearches with a combination of k-nearest and circle, so that only the k-nearest points within the circle radius are retained\n"
                        "\"next\"\n"
                        "  \tsearches only the next nearest edge (within the nearest and adjacent as well as reversed edges),\n"
                        "  \tvery fast but often inaccurate except for high precision tracks")
                ("radius", po::value<double>(&data.radius)->default_value(200.0, "200.0"),
                        "radius in meters for candidate search when circle or combined algorithm is selected, "
                        "is automatically doubled each time when no candidates within given radius are found, "
                        "radius is starting radius if adaptive-radius is enabled (which is by default)")
                ("radius-upper-limit", po::value<double>(&data.radius_upper_limit)->default_value(10000.0, "10000.0"),
                        "radius upper limit in meters for candidate search when adaptive-radius is used, "
                        "eventually stops automatically doubling of radius when no candidates are found")
                ("radius-lower-limit", po::value<double>(&data.radius_lower_limit)->default_value(200.0, "200.0"),
                        "radius lower limit in meters for candidate search when adaptive-radius is used, "
                        "prevents too small search radii in areas with dense track measurements")
                ("adaptive-radius", po::value<bool>(&data.adaptive_radius)->default_value(true, "on"),
                        "adaptive-radius is only used when circle or combined algorithm is selected, "
                        "the radius is then reduced to the minimum distance to the next and previous candidate, "
                        "the radius-lower-limit is never undershot, when no candidate is found, the radius is doubled until radius-upper-limit, "
                        "this reduces the amount of candidates in dense road network areas, it works best with candidate-adoption enabled")
                ("k-nearest", po::value<std::size_t>(&data.k_nearest)->default_value(16),
                        "k-nearest edges to search for in candidate search when nearest or combined algorithm is selected");

        return options;
    }

    po::options_description options_model(model_data &data) {
        po::options_description options("Model Options", global.console.cols);

        options.add_options()
                ("model", po::value<std::string>(&data.model)->default_value("value-iteration"),
                        "model to use, possible options:\n"
                        "\"policy-iteration\"\n"
                        "  \tuses policy iteration algorithm with markov decision process\n"
                        "\"value-iteration\"\n"
                        "  \tuses value iteration algorithm with markov decision process\n"
                        "\"q-learning\"\n"
                        "  \tuses Q-learning algorithm with markov decision process\n"
                        "\"viterbi\"\n"
                        "  \tuses Viterbi algorithm with hidden markov model")
                ("skip-errors", po::value<bool>(&data.skip_errors)->default_value(true, "on"),
                        "enable skipping high error situations in markov decision processes, "
                        "leads to gaps in the exported track, but this is much better than high errors")
                ("state-size", po::value<std::size_t>(&data.state_size)->default_value(2),
                        "amount of adjacent track points to consider for a state in value iteration and q-learning, "
                        "default value of 2 means two directly succeeding points, value of 3 means three directly succeeding points, "
                        "with 3 succeeding points, additional metrics for direction changes and zig-zag movements are processed, "
                        "3 has a huge impact on calculation speed because the state space becomes very large, "
                        "other values are not permitted, also does not work with viterbi algorithm due to design principles")
                ("discount", po::value<double>(&data.discount)->default_value(0.99, "0.99"),
                        "discount factor for value-iteration and q-learning, "
                        "discount of 0.0 means only the current state is of value, "
                        "discount of 1.0 means all state sequences up to the goal are of value fur the current state, "
                        "defaults to 0.99 for making sure that loops eventually unroll");

        return options;
    }

    po::options_description options_weight_mdp(weight_mdp_data &data) {
        po::options_description options("MDP Weight Options", global.console.cols);

        options.add_options()
                ("mdp-distance-factor", po::value<double>(&data.distance_factor)->default_value(0.8, "0.8"),
                        "distance factor for rewarding distances between measurement point and candidate position")
                ("mdp-length-factor", po::value<double>(&data.length_factor)->default_value(0.3, "0.3"),
                        "length factor for rewarding length differences between track segments and routes in network")
                ("mdp-azimuth-factor", po::value<double>(&data.azimuth_factor)->default_value(0.5, "0.5"),
                        "azimuth factor for rewarding azimuth differences between track segments and routes in network")
                ("mdp-direction-factor", po::value<double>(&data.direction_factor)->default_value(0.6, "0.6"),
                        "direction factor for rewarding direction changes of routes in network")
                ("mdp-turn-factor", po::value<double>(&data.turn_factor)->default_value(0.5, "0.5"),
                        "turn factor for rewarding turn differences between track segments and routes in network");

        return options;
    }

    po::options_description options_weight_hmm(weight_hmm_data &data) {
        po::options_description options("HMM Weight Options", global.console.cols);

        options.add_options()
                ("hmm-distance-factor", po::value<double>(&data.distance_factor)->default_value(0.8, "0.8"),
                        "distance factor for probability of distances between measurement point and candidate position")
                ("hmm-length-factor", po::value<double>(&data.length_factor)->default_value(0.1, "0.1"),
                        "length factor for probability of length differences between track segments and routes in network")
                ("hmm-azimuth-factor", po::value<double>(&data.azimuth_factor)->default_value(0.2, "0.2"),
                        "azimuth factor for probability of azimuth differences between track segments and routes in network")
                ("hmm-direction-factor", po::value<double>(&data.direction_factor)->default_value(0.8, "0.8"),
                        "direction factor for probability of direction changes of routes in network")
                ("hmm-turn-factor", po::value<double>(&data.turn_factor)->default_value(0.6, "0.6"),
                        "turn factor for rewarding turn differences between track segments and routes in network");

        return options;
    }

    po::options_description options_dynamic_programming(dynamic_programming_data &data) {
        po::options_description options("Dynamic Programming Options", global.console.cols);

        options.add_options()
                ("threshold", po::value<double>(&data.threshold)->default_value(1e-3, "1e-3"),
                        "threshold for stopping iterations in policy- and value-iteration when the previous round improvement was below the given threshold");

        return options;
    }

    po::options_description options_learning(learning_data &data) {
        po::options_description options("Learning Options", global.console.cols);

        options.add_options()
                ("learning-rate", po::value<double>(&data.learning_rate)->default_value(0.9, "0.9"),
                        "learning rate for q-learning, only applies if q-learning model is selected, "
                        "higher learning-rate means faster learning, "
                        "lower learning-rate means slower learning")
                ("epsilon", po::value<double>(&data.epsilon)->default_value(0.5, "0.5"),
                        "epsilon random action selection value for q-learning, only applies if q-learning model is selected, "
                        "epsilon of 0.0 means no random action selection, always choose next best, "
                        "epsilon of 1.0 means always random selection, never select next best")
                ("early-stop-factor", po::value<double>(&data.early_stop_factor)->default_value(1.0, "1.0"),
                        "factor for multiplication with early-stopping counter (equals episodes), "
                        "early-stopping episodes are calculated from sum of candidates, so depends on candidate search settings, "
                        "early-stop-factor of 0.0 means immediate stopping after one improvement episode, "
                        "early-stop-factor of 1.0 means early-stop only after sum of candidates episodes, "
                        "a smaller factor leads to faster convergence but worse results, "
                        "a larger factor (even larger than 1.0) leads to better results but needs more time for convergence")
                ("max-episodes", po::value<std::size_t>(&data.episodes)->default_value(1000000000),
                        "maximum episodes for definitive stop in q-learning, do not change except you know what you are doing, "
                        "training is generally stopped early before max-episodes is ever reached, "
                        "this is calculated from the found candidates, the more possible candidates, the longer the training");

        return options;
    }

    po::options_description options_comparison(comparison_data &data) {
        auto tracks = options_tracks(data, "compare-");

        po::options_description options("Comparison Options", global.console.cols);

        options.add_options()
                ("ignore-non-existent", po::bool_switch(&data.ignore_non_existent),
                        "ignore comparison tracks that do not exist in the matches,"
                        "without this option, an empty match is presumed for not existing matches, "
                        "which decreases the overall correct fraction")
                ("compare-simplify-distance-tolerance",
                        po::value<double>(&data.simplifying_tolerance)->default_value(0.1, "0.1"),
                        "compare simplify distance tolerance in meters around comparison lines for Douglas-Peucker algorithm")
                ("compare-simplify-reverse-tolerance",
                        po::value<double>(&data.simplifying_reverse_tolerance)->default_value(0.1, "0.1"),
                        "compare reverse detection azimuth tolerance in degrees for splitting simplifications")
                ("compare-adoption-distance-tolerance",
                        po::value<double>(&data.adoption_distance_tolerance)->default_value(0.1, "0.1"),
                        "compare adoption distance tolerance in meters around a point for merging close points to remove tiny differences")
                ("compare-split-distance-tolerance",
                        po::value<double>(&data.split_distance_tolerance)->default_value(1.0, "1.0"),
                        "compare split distance tolerance in meters for splitting lines at closest points between lines")
                ("compare-split-direction-tolerance",
                        po::value<double>(&data.split_direction_tolerance)->default_value(90.0, "90.0"),
                        "compare split direction tolerance in degrees for allowing only splits at points "
                        "that have no more direction change degrees than specified");

        tracks.add(options);

        return tracks;
    }

    po::options_description options_mode(mode_data &data, const MODE mode) {
        auto _all = options_all(data.all);
        auto _global = options_global(data.global);

        _all.add(_global);

        return _all;
    }

    po::options_description options_prepare(prepare_data &data) {
        auto _mode = options_mode(data, MODE::PREPARE);

        auto _prepare_network = options_prepare_network(data.network);
        auto _prepare_osm_import = options_prepare_osm_import(data.osm_import);
        auto _prepare_srs = options_prepare_srs(data.srs);
        auto _prepare_files = options_prepare_files(data.files);
        auto _prepare_export = options_prepare_export(data.expo);
        auto _prepare_process = options_prepare_process(data.process);
        auto _prepare_memory = options_prepare_memory(data.memory);

        _mode.add(_prepare_network);
        _mode.add(_prepare_osm_import);
        _mode.add(_prepare_srs);
        _mode.add(_prepare_files);
        _mode.add(_prepare_export);
        _mode.add(_prepare_process);
        _mode.add(_prepare_memory);

        return _mode;
    }

    po::options_description options_work(work_data &data, const MODE mode) {
        auto _mode = options_mode(data, mode);

        auto _console = options_console(data.console);
        auto _performance = options_performance(data.performance);

        _mode.add(_console);
        _mode.add(_performance);

        return _mode;
    }

    po::options_description options_match(match_data &data) {
        auto _work = options_work(data, MODE::MATCH);

        auto _input = options_input(data.input);
        auto _srs = options_srs(data.srs, "network-");
        auto _files = options_files(data.files);
        auto _memory = options_memory(data.memory);
        auto _match_output = options_match_output(data.match_output);
        auto _tracks = options_tracks(data.tracks);
        auto _srs_tracks = options_srs_transform(data.srs_tracks, "tracks-");
        auto _csv = options_csv_tracks(data.csv);
        auto _expo = options_export(data.expo);
        auto _matching = options_matching(data.matching);
        auto _model = options_model(data.model);
        auto _weights_mdp = options_weight_mdp(data.weights_mdp);
        auto _weights_hmm = options_weight_hmm(data.weights_hmm);
        auto _dynamic_programming = options_dynamic_programming(data.dynamic_programming);
        auto _learning = options_learning(data.learning);

        _work.add(_input);
        _work.add(_srs);
        _work.add(_files);
        _work.add(_memory);
        _work.add(_match_output);
        _work.add(_tracks);
        _work.add(_srs_tracks);
        _work.add(_csv);
        _work.add(_expo);
        _work.add(_matching);
        _work.add(_model);
        _work.add(_weights_mdp);
        _work.add(_weights_hmm);
        _work.add(_dynamic_programming);
        _work.add(_learning);

        return _work;
    }

    po::options_description options_compare(compare_data &data) {
        auto _work = options_work(data, MODE::COMPARE);

        auto _compare_output = options_compare_output(data.compare_output);
        auto _tracks = options_tracks(data.match, "match-");
        auto _srs_tracks = options_srs_transform(data.srs_match, "match-");
        auto _csv = options_csv_tracks(data.csv_match, "match-");
        auto _comparison = options_comparison(data.comparison);
        auto _srs_compare = options_srs_transform(data.srs_compare, "compare-");
        auto _csv_compare = options_csv_tracks(data.csv_compare, "compare-");

        _work.add(_compare_output);
        _work.add(_tracks);
        _work.add(_srs_tracks);
        _work.add(_csv);
        _work.add(_comparison);
        _work.add(_srs_compare);
        _work.add(_csv_compare);

        return _work;
    }

    void parse_cli(int argc, char *argv[], po::variables_map &vm, po::options_description &all, const MODE mode) {
        auto cli_parser =
                po::basic_command_line_parser(argc, argv).options(all).style(0).extra_parser(po::ext_parser());
        if (mode == MODE::NONE) {
            cli_parser = cli_parser.allow_unregistered();
        }
        po::store(cli_parser.run(), vm);
    }

    void parse_file(po::variables_map &vm, po::options_description &all, const MODE mode) {
        std::filesystem::path path{vm["config"].as<std::string>()};
        if (not vm["config"].defaulted() or std::filesystem::exists(path)) {
            // if value is set, file needs to exist; else check if file exists before proceeding
            const std::string path_str = path.string();
            po::store(po::parse_config_file(path_str.c_str(), all, mode == MODE::NONE), vm);
        }
    }

    bool _show_help_checker(const po::variables_map &vm, const MODE mode) {
        bool _prepare = vm.count("prepare") and vm["prepare"].as<bool>();
        bool _match = vm.count("match") and vm["match"].as<bool>();
        bool _server = vm.count("server") and vm["server"].as<bool>();
        bool _compare = vm.count("compare") and vm["compare"].as<bool>();

        return (mode == MODE::NONE and not _prepare and not _match and not _server and not _compare) or
                (mode == MODE::PREPARE and _prepare) or
                (mode == MODE::MATCH and _match) or
                (mode == MODE::SERVER and _server) or
                (mode == MODE::COMPARE and _compare);
    }

    bool show_help(const po::options_description &all, const po::variables_map &vm, const MODE mode) {
        if (vm.count("help") and _show_help_checker(vm, mode)) {
            std::cout << all << std::endl;
            return true;
        }
        return false;
    }

    bool show_version(const po::variables_map &vm) {
        if (vm.count("version")) {
            std::cout << "Version: " << util::version() << std::endl;
            return true;
        }
        return false;
    }

    void show_error(const std::exception &e, const po::options_description &all) {
        std::clog << "Error:\n" << e.what() << std::endl;
        std::cout << "\n" << "Use --help for more information." << std::endl;
    }

    bool process(int argc, char *argv[], po::variables_map &vm, po::options_description &all, const MODE mode) {
        try {
            parse_cli(argc, argv, vm, all, mode);
            parse_file(vm, all, mode);

            if (show_help(all, vm, mode)) {
                return false;
            }

            if (show_version(vm)) {
                return false;
            }

            po::notify(vm);
        } catch (std::exception &e) {
            show_error(e, all);
            return false;
        }

        return true;
    }

}
