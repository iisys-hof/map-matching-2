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

#ifndef MAP_MATCHING_2_APP_NETWORK_HPP
#define MAP_MATCHING_2_APP_NETWORK_HPP

#include <string>
#include <vector>
#include <filesystem>

#include "types/geometry/network/network.hpp"
#include "types/geometry/network/network_import.hpp"
#include "types/io/network/osm_handler.hpp"

#include "geometry/network/traits/network.hpp"

#include "io/serialization/serializer.hpp"
#include "io/serialization/deserializer.hpp"

#include "options.hpp"

namespace map_matching_2::app {

    enum START_METHOD {
        DO_NOT_IMPORT,
        DO_IMPORT
    };

    template<typename TagHelper>
    [[nodiscard]] auto _create_osm_tag_helper(const prepare_data &data) {
        using tag_helper_type = TagHelper;

        if constexpr (io::helper::is_mmap_tag_helper<tag_helper_type>) {
            using tag_storage_type = typename tag_helper_type::tag_storage_type;

            return tag_helper_type{
                    tag_storage_type{
                            data.network.output, data.files.osm_tags_file,
                            io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                            data.files.initial_size,
                            io::memory_mapped::storage::destroy_on_close_tag::DESTROY_ON_CLOSE
                    }
            };
        } else if constexpr (io::helper::is_memory_tag_helper<tag_helper_type>) {
            return tag_helper_type{};
        } else {
            static_assert(util::dependent_false_v<tag_helper_type>,
                    "TagHelper is neither from type memory or mmap.");
        }
    }

    template<typename TagHelper>
    [[nodiscard]] auto _create_import_tag_helper(const prepare_data &data) {
        using tag_helper_type = TagHelper;

        if constexpr (io::helper::is_mmap_tag_helper<tag_helper_type>) {
            using tag_storage_type = typename tag_helper_type::tag_storage_type;

            return tag_helper_type{
                    tag_storage_type{
                            data.network.output, data.files.tags_import_file,
                            io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                            data.files.initial_size,
                            io::memory_mapped::storage::destroy_on_close_tag::DESTROY_ON_CLOSE
                    }
            };
        } else if constexpr (io::helper::is_memory_tag_helper<tag_helper_type>) {
            return tag_helper_type{};
        } else {
            static_assert(util::dependent_false_v<tag_helper_type>,
                    "TagHelper is neither from type memory or mmap.");
        }
    }

    template<typename TagHelper>
    [[nodiscard]] auto _create_tag_helper(const prepare_data &data) {
        using tag_helper_type = TagHelper;

        if constexpr (io::helper::is_mmap_tag_helper<tag_helper_type>) {
            using tag_storage_type = typename tag_helper_type::tag_storage_type;

            return tag_helper_type{
                    tag_storage_type{
                            data.network.output, data.files.tags_file,
                            io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                            data.files.initial_size,
                            io::memory_mapped::storage::destroy_on_close_tag::DO_NOT_DESTROY_ON_CLOSE,
                            io::memory_mapped::storage::shrink_on_close_tag::SHRINK_ON_CLOSE
                    }
            };
        } else if constexpr (io::helper::is_memory_tag_helper<tag_helper_type>) {
            return tag_helper_type{};
        } else {
            static_assert(util::dependent_false_v<tag_helper_type>,
                    "TagHelper is neither from type memory or mmap.");
        }
    }

    template<typename TagHelper>
    [[nodiscard]] auto _open_tag_helper(const match_data &data) {
        using tag_helper_type = TagHelper;

        if constexpr (io::helper::is_mmap_tag_helper<tag_helper_type>) {
            using tag_storage_type = typename tag_helper_type::tag_storage_type;

            return tag_helper_type{
                    tag_storage_type{
                            data.input.input, data.files.tags_file,
                            io::memory_mapped::storage::open_or_create_tag::OPEN_READ_ONLY
                    }
            };
        } else if constexpr (io::helper::is_memory_tag_helper<tag_helper_type>) {
            return tag_helper_type{};
        } else {
            static_assert(util::dependent_false_v<tag_helper_type>,
                    "TagHelper is neither from type memory or mmap.");
        }
    }

    template<typename IndexBuildHelper>
    [[nodiscard]] auto _create_index_build_helper(const prepare_data &data) {
        using index_build_helper_type = IndexBuildHelper;

        if constexpr (io::helper::is_mmap_index_build_helper<IndexBuildHelper>) {
            using index_build_storage_type = typename index_build_helper_type::index_build_storage_type;

            return index_build_helper_type{
                    index_build_storage_type{
                            data.network.output, data.files.index_build_file,
                            io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                            data.files.initial_size,
                            io::memory_mapped::storage::destroy_on_close_tag::DESTROY_ON_CLOSE,
                            io::memory_mapped::storage::shrink_on_close_tag::DO_NOT_SHRINK_ON_CLOSE
                    }
            };
        } else if constexpr (io::helper::is_memory_index_build_helper<index_build_helper_type>) {
            return index_build_helper_type{};
        } else {
            static_assert(util::dependent_false_v<index_build_helper_type>,
                    "IndexBuildHelper is neither from type memory or mmap.");
        }
    }

    template<typename IndexHelper>
    [[nodiscard]] auto _create_index_helper(const prepare_data &data) {
        using index_helper_type = IndexHelper;

        using index_build_helper_type = typename index_helper_type::index_build_helper_type;
        auto index_build_helper = _create_index_build_helper<index_build_helper_type>(data);

        if constexpr (io::helper::is_mmap_index_helper<IndexHelper>) {
            using index_storage_type = typename index_helper_type::index_storage_type;

            return index_helper_type{
                    index_storage_type{
                            data.network.output, data.files.index_file,
                            io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                            data.files.initial_size,
                            io::memory_mapped::storage::destroy_on_close_tag::DO_NOT_DESTROY_ON_CLOSE,
                            io::memory_mapped::storage::shrink_on_close_tag::SHRINK_ON_CLOSE
                    },
                    std::move(index_build_helper)
            };
        } else if constexpr (io::helper::is_memory_index_helper<index_helper_type>) {
            using index_storage_type = typename index_helper_type::index_storage_type;

            return index_helper_type{
                    index_storage_type{},
                    std::move(index_build_helper)
            };
        } else {
            static_assert(util::dependent_false_v<index_helper_type>,
                    "IndexHelper is neither from type memory or mmap.");
        }
    }

    template<typename IndexHelper>
    [[nodiscard]] auto _open_index_helper(const match_data &data) {
        using index_helper_type = IndexHelper;
        using index_build_helper_type = typename index_helper_type::index_build_helper_type;

        if constexpr (io::helper::is_mmap_index_helper<IndexHelper>) {
            using index_storage_type = typename index_helper_type::index_storage_type;

            return index_helper_type{
                    index_storage_type{
                            data.input.input, data.files.index_file,
                            io::memory_mapped::storage::open_or_create_tag::OPEN_READ_ONLY
                    },
                    index_build_helper_type{}
            };
        } else if constexpr (io::helper::is_memory_index_helper<index_helper_type>) {
            using index_storage_type = typename index_helper_type::index_storage_type;

            return index_helper_type{
                    index_storage_type{},
                    index_build_helper_type{}
            };
        } else {
            static_assert(util::dependent_false_v<index_helper_type>,
                    "IndexHelper is neither from type memory or mmap.");
        }
    }

    template<typename NetworkImport>
    [[nodiscard]] auto _create_network_import(const prepare_data &data) {
        using network_type = NetworkImport;
        using graph_helper_type = typename geometry::network::network_traits<network_type>::graph_helper_type;
        using tag_helper_type = typename geometry::network::network_traits<network_type>::tag_helper_type;

        auto tag_helper_import = _create_import_tag_helper<tag_helper_type>(data);

        if constexpr (io::helper::is_mmap_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{
                                    data.network.output, data.files.graph_import_file,
                                    io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                                    data.files.initial_size,
                                    io::memory_mapped::storage::destroy_on_close_tag::DESTROY_ON_CLOSE
                            },
                            std::move(tag_helper_import)
                    });
        } else if constexpr (io::helper::is_memory_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{},
                            std::move(tag_helper_import)
                    });
        } else {
            static_assert(util::dependent_false_v<graph_helper_type>,
                    "GraphHelper is neither from type memory or mmap.");
        }
    }

    template<typename Network>
    [[nodiscard]] auto _create_network(const prepare_data &data) {
        using network_type = Network;
        using graph_helper_type = typename geometry::network::network_traits<network_type>::graph_helper_type;
        using tag_helper_type = typename geometry::network::network_traits<network_type>::tag_helper_type;

        auto tag_helper = _create_tag_helper<tag_helper_type>(data);

        using index_helper_type = typename network_type::index_helper_type;
        auto index_helper = _create_index_helper<index_helper_type>(data);

        if constexpr (io::helper::is_mmap_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{
                                    data.network.output, data.files.graph_file,
                                    io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                                    data.files.initial_size,
                                    io::memory_mapped::storage::destroy_on_close_tag::DO_NOT_DESTROY_ON_CLOSE,
                                    io::memory_mapped::storage::shrink_on_close_tag::SHRINK_ON_CLOSE
                            },
                            std::move(tag_helper)
                    },
                    std::move(index_helper));
        } else if constexpr (io::helper::is_memory_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{},
                            std::move(tag_helper)
                    },
                    std::move(index_helper));
        } else {
            static_assert(util::dependent_false_v<graph_helper_type>,
                    "GraphHelper is neither from type memory or mmap.");
        }
    }

    template<typename Network>
    [[nodiscard]] auto _open_network(const match_data &data) {
        using network_type = Network;
        using graph_helper_type = typename geometry::network::network_traits<network_type>::graph_helper_type;
        using tag_helper_type = typename geometry::network::network_traits<network_type>::tag_helper_type;

        auto tag_helper = _open_tag_helper<tag_helper_type>(data);

        using index_helper_type = typename network_type::index_helper_type;
        auto index_helper = _open_index_helper<index_helper_type>(data);

        if constexpr (io::helper::is_mmap_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{
                                    data.input.input, data.files.graph_file,
                                    io::memory_mapped::storage::open_or_create_tag::OPEN_READ_ONLY
                            },
                            std::move(tag_helper)
                    },
                    std::move(index_helper));
        } else if constexpr (io::helper::is_memory_graph_helper<graph_helper_type>) {
            using graph_storage_type = typename graph_helper_type::graph_storage_type;

            return std::make_unique<network_type>(
                    graph_helper_type{
                            graph_storage_type{},
                            std::move(tag_helper)
                    },
                    std::move(index_helper));
        } else {
            static_assert(util::dependent_false_v<graph_helper_type>,
                    "GraphHelper is neither from type memory or mmap.");
        }
    }

    template<typename OSMHandlerHelper, typename GraphHelper>
    [[nodiscard]] auto _create_osm_handler(GraphHelper &graph_helper, const prepare_data &data) {
        using osm_handler_helper_type = OSMHandlerHelper;

        using tag_helper_type = typename osm_handler_helper_type::tag_helper_type;
        auto osm_tag_helper = _create_osm_tag_helper<tag_helper_type>(data);

        auto reprojector_variant = geometry::create_point_reprojector(data.srs.srs_transform);

        if constexpr (io::helper::is_mmap_osm_handler_helper<osm_handler_helper_type>) {
            using osm_handler_storage_type = typename osm_handler_helper_type::osm_handler_storage_type;
            using osm_handler_type = io::network::osm_handler<GraphHelper,
                io::network::osm_mmap_index_type,
                osm_handler_helper_type>;

            return std::make_unique<osm_handler_type>(
                    graph_helper, std::move(reprojector_variant), osm_handler_helper_type{
                            osm_handler_storage_type{
                                    data.network.output, data.files.osm_vertices_file,
                                    io::memory_mapped::storage::open_or_create_tag::DESTROY_AND_CREATE,
                                    data.files.initial_size,
                                    io::memory_mapped::storage::destroy_on_close_tag::DESTROY_ON_CLOSE
                            },
                            std::move(osm_tag_helper)
                    });
        } else if constexpr (io::helper::is_memory_osm_handler_helper<osm_handler_helper_type>) {
            using osm_handler_storage_type = typename osm_handler_helper_type::osm_handler_storage_type;
            using osm_handler_type = io::network::osm_handler<GraphHelper,
                io::network::osm_memory_index_type, osm_handler_helper_type>;

            return std::make_unique<osm_handler_type>(
                    graph_helper, std::move(reprojector_variant), osm_handler_helper_type{
                            osm_handler_storage_type{},
                            std::move(osm_tag_helper)
                    });
        } else {
            static_assert(util::dependent_false_v<osm_handler_helper_type>,
                    "OSMHandlerHelper is neither from type memory or mmap.");
        }
    }

    [[nodiscard]] geometry::network::network_import_variant_type create_network_import(const prepare_data &data);

    [[nodiscard]] geometry::network::network_variant_type open_network(const match_data &data);

    template<typename CoordinateSystem, typename Graph, typename Tags, typename Indices, typename BuildIndices>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    _create_compatible_network_build_indices(const prepare_data &data) {
        return _create_network<geometry::network::network<
            io::helper::graph_helper_type<geometry::point_type<CoordinateSystem>,
                Graph, Tags>,
            io::helper::index_helper_type<geometry::point_type<CoordinateSystem>,
                Indices, BuildIndices>>>(data);
    }

    template<typename CoordinateSystem, typename Graph, typename Tags, typename Indices>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    _create_compatible_network_indices(const prepare_data &data) {
        if (data.memory.mmap_index_build) {
            return _create_compatible_network_build_indices<CoordinateSystem, Graph, Tags, Indices,
                io::memory_mapped::file_types>(data);
        } else {
            return _create_compatible_network_build_indices<CoordinateSystem, Graph, Tags, Indices,
                io::memory_mapped::memory_types>(data);
        }
    }

    template<typename CoordinateSystem, typename Graph, typename Tags>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    _create_compatible_network_tags(const prepare_data &data) {
        if (data.memory.mmap_indices) {
            return _create_compatible_network_indices<CoordinateSystem, Graph, Tags,
                io::memory_mapped::file_types>(data);
        } else {
            return _create_compatible_network_indices<CoordinateSystem, Graph, Tags,
                io::memory_mapped::memory_types>(data);
        }
    }

    template<typename CoordinateSystem, typename Graph>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    _create_compatible_network_graph(const prepare_data &data) {
        if (data.memory.mmap_tags) {
            return _create_compatible_network_tags<CoordinateSystem, Graph,
                io::helper::tag_helper_type<io::memory_mapped::file_types>>(data);
        } else {
            return _create_compatible_network_tags<CoordinateSystem, Graph,
                io::helper::tag_helper_type<io::memory_mapped::memory_types>>(data);
        }
    }

    template<typename CoordinateSystem>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    _create_compatible_network_coordinate_system(const prepare_data &data) {
        if (data.memory.mmap_graph) {
            return _create_compatible_network_graph<CoordinateSystem, io::memory_mapped::file_types>(data);
        } else {
            return _create_compatible_network_graph<CoordinateSystem, io::memory_mapped::memory_types>(data);
        }
    }

    template<typename CoordinateSystem>
    [[nodiscard]] geometry::network::network_variant_compatible_type<CoordinateSystem>
    create_compatible_network(const prepare_data &data) {
        return _create_compatible_network_coordinate_system<CoordinateSystem>(data);
    }

    template<typename GraphHelper, typename CoordinateSystem, typename OSM, typename Tags>
    [[nodiscard]] io::network::osm_handler_variant_type<GraphHelper>
    _create_osm_handler_tags(GraphHelper &graph_helper, const prepare_data &data) {
        return _create_osm_handler<io::helper::osm_handler_helper_type<
            geometry::point_type<CoordinateSystem>, OSM, Tags>>(graph_helper, data);
    }

    template<typename GraphHelper, typename CoordinateSystem, typename OSM>
    [[nodiscard]] io::network::osm_handler_variant_type<GraphHelper>
    _create_osm_handler_osm(GraphHelper &graph_helper, const prepare_data &data) {
        if (data.memory.mmap_osm_tags) {
            return _create_osm_handler_tags<GraphHelper, CoordinateSystem, OSM,
                io::helper::import_tag_helper_type<io::memory_mapped::file_types>>(graph_helper, data);
        } else {
            return _create_osm_handler_tags<GraphHelper, CoordinateSystem, OSM,
                io::helper::import_tag_helper_type<io::memory_mapped::memory_types>>(graph_helper, data);
        }
    }

    template<typename GraphHelper, typename CoordinateSystem>
    [[nodiscard]] io::network::osm_handler_variant_type<GraphHelper>
    _create_osm_handler_coordinate_system(GraphHelper &graph_helper, const prepare_data &data) {
        if (data.memory.mmap_osm_vertices) {
            return _create_osm_handler_osm<GraphHelper, CoordinateSystem, io::memory_mapped::file_types>(
                    graph_helper, data);
        } else {
            return _create_osm_handler_osm<GraphHelper, CoordinateSystem, io::memory_mapped::memory_types>(
                    graph_helper, data);
        }
    }

    template<typename GraphHelper>
    [[nodiscard]] io::network::osm_handler_variant_type<GraphHelper>
    create_osm_handler(GraphHelper &graph_helper, const prepare_data &data) {
        return _create_osm_handler_coordinate_system<GraphHelper,
            typename geometry::data<typename GraphHelper::vertex_data_type::point_type>::coordinate_system_type>(
                graph_helper, data);
    }

    template<typename Network>
    void save_network(const Network &network, const prepare_data &data) {
        if constexpr (io::helper::is_memory_tag_helper<
            typename geometry::network::network_traits<Network>::tag_helper_type>) {
            if (not data.memory.mmap_tags) {
                auto _tags_file = std::filesystem::path{data.network.output} / data.files.tags_file;
                io::serialization::serializer tags_serializer{_tags_file.string(), network.tag_helper()};
                tags_serializer.write();
            }
        }

        if constexpr (io::helper::is_memory_index_helper<
            typename Network::index_helper_type>) {
            if (not data.memory.mmap_indices) {
                auto _index_file = std::filesystem::path{data.network.output} / data.files.index_file;
                io::serialization::serializer index_serializer{_index_file.string(), network.index_helper()};
                index_serializer.write();
            }
        }

        if constexpr (io::helper::is_memory_graph_helper<
            typename geometry::network::network_traits<Network>::graph_helper_type>) {
            if (not data.memory.mmap_graph) {
                auto _graph_file = std::filesystem::path{data.network.output} / data.files.graph_file;
                io::serialization::serializer graph_serializer{_graph_file.string(), network.graph_helper()};
                graph_serializer.write();
            }
        }
    }

    template<typename Network>
    void load_network(Network &network, const match_data &data) {
        if constexpr (io::helper::is_memory_tag_helper<
            typename geometry::network::network_traits<Network>::tag_helper_type>) {
            if (not data.memory.mmap_tags) {
                auto _tags_file = std::filesystem::path{data.input.input} / data.files.tags_file;
                io::serialization::deserializer tags_deserializer{_tags_file.string(), network.tag_helper()};
                tags_deserializer.read();
            }
        }

        if constexpr (io::helper::is_memory_index_helper<
            typename Network::index_helper_type>) {
            if (not data.memory.mmap_indices) {
                auto _index_file = std::filesystem::path{data.input.input} / data.files.index_file;
                io::serialization::deserializer index_deserializer{_index_file.string(), network.index_helper()};
                index_deserializer.read();
            }
        }

        if constexpr (io::helper::is_memory_graph_helper<
            typename geometry::network::network_traits<Network>::graph_helper_type>) {
            if (not data.memory.mmap_graph) {
                auto _graph_file = std::filesystem::path{data.input.input} / data.files.graph_file;
                io::serialization::deserializer graph_deserializer{_graph_file.string(), network.graph_helper()};
                graph_deserializer.read();
            }
        }
    }

}

#endif //MAP_MATCHING_2_APP_NETWORK_HPP
