# library options
option(ENABLE_ZLIB "Enable adding library zlib" ON)
option(ENABLE_BZ2 "Enable adding library bz2" ON)
option(ENABLE_EXPAT "Enable adding library expat" ON)
option(ENABLE_PROTOBUF "Enable adding library protobuf" ON)
option(ENABLE_PROTOZERO "Enable adding library protozero" ON)
option(ENABLE_DATE "Enable adding library date" ON)
option(ENABLE_CSV_PARSER "Enable adding library csv-parser" ON)
option(ENABLE_GPX "Enable adding library gpx" ON)
option(ENABLE_RPMALLOC "Enable adding library rpmalloc" ON)
option(ENABLE_BOOST "Enable adding library Boost" ON)
option(ENABLE_LIBOSMIUM "Enable adding library libosmium" ON)

include(FetchContent)
message(STATUS "Adding external libraries")

if (ENABLE_ZLIB)
    message(STATUS "Adding library: zlib")
    include(zlib/CMakeLists.txt)
endif ()

if (ENABLE_BZ2)
    message(STATUS "Adding library: bz2")
    include(bz2/CMakeLists.txt)
endif ()

if (ENABLE_EXPAT)
    message(STATUS "Adding library: expat")
    include(expat/CMakeLists.txt)
endif ()

if (ENABLE_PROTOBUF)
    message(STATUS "Adding library: protobuf")
    include(protobuf/CMakeLists.txt)
endif ()

if (ENABLE_PROTOZERO)
    message(STATUS "Adding library: protozero")
    include(protozero/CMakeLists.txt)
endif ()

if (ENABLE_DATE)
    message(STATUS "Adding library: date")
    include(date/CMakeLists.txt)
endif ()

if (ENABLE_CSV_PARSER)
    message(STATUS "Adding library: csv_parser")
    include(csv_parser/CMakeLists.txt)
endif ()

if (ENABLE_GPX)
    message(STATUS "Adding library: gpx")
    include(gpx/CMakeLists.txt)
endif ()

if (ENABLE_RPMALLOC)
    message(STATUS "Adding library: rpmalloc")
    include(rpmalloc/CMakeLists.txt)
endif ()

if (ENABLE_BOOST)
    message(STATUS "Adding library: boost")
    include(boost/CMakeLists.txt)
endif ()

if (ENABLE_LIBOSMIUM)
    message(STATUS "Adding library: libosmium")
    include(libosmium/CMakeLists.txt)
endif ()

message(STATUS "Adding external libraries - SUCCESS")
