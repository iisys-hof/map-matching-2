include(FetchContent)

function(Managed_FetchContent NAME)
    set(oneValueArgs
            URL
            URL_HASH
            GIT_REPOSITORY
            GIT_TAG
    )

    cmake_parse_arguments(ARG "" "${oneValueArgs}" "" ${ARGN})

    set(source_dir "${FETCHCONTENT_BASE_DIR}/${NAME}-src")
    set(binary_dir "${FETCHCONTENT_BASE_DIR}/${NAME}-build")
    set(tmp_dir "${FETCHCONTENT_BASE_DIR}/${NAME}-tmp")

    set(stamp_dir "${CMAKE_BINARY_DIR}/_stamps")
    set(stamp_file "${stamp_dir}/${NAME}-stamp.txt")

    set(current_stamp
            "NAME=${NAME}\n"
            "URL=${ARG_URL}\n"
            "URL_HASH=${ARG_URL_HASH}\n"
            "GIT_REPOSITORY=${ARG_GIT_REPOSITORY}\n"
            "GIT_TAG=${ARG_GIT_TAG}\n"
            "CMAKE_CXX_COMPILER_ID=${CMAKE_CXX_COMPILER_ID}\n"
            "CMAKE_CXX_COMPILER_VERSION=${CMAKE_CXX_COMPILER_VERSION}\n"
            "CMAKE_GENERATOR=${CMAKE_GENERATOR}\n"
            "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}\n"
    )
    string(REPLACE ";" "" current_stamp "${current_stamp}")

    set(needs_refresh OFF)

    if (EXISTS "${stamp_file}")
        file(READ "${stamp_file}" previous_stamp)

        if (NOT previous_stamp STREQUAL current_stamp)
            message(STATUS "Dependency '${NAME}' needs refresh because its version changed")
            set(needs_refresh ON)
        endif ()
    else ()
        set(needs_refresh ON)
    endif ()

    if (needs_refresh)
        file(REMOVE_RECURSE
                "${source_dir}"
                "${binary_dir}"
                "${tmp_dir}")

        file(MAKE_DIRECTORY "${stamp_dir}")
        file(WRITE "${stamp_file}" ${current_stamp})
    else ()
        set(FETCHCONTENT_FULLY_DISCONNECTED ON)
    endif ()

    FetchContent_Declare(${NAME}
            ${ARGN})

    FetchContent_MakeAvailable(${NAME})

    set(${NAME}_SOURCE_DIR "${${NAME}_SOURCE_DIR}" PARENT_SCOPE)
    set(${NAME}_BINARY_DIR "${${NAME}_BINARY_DIR}" PARENT_SCOPE)
    set(${NAME}_POPULATED "${${NAME}_POPULATED}" PARENT_SCOPE)
    set(${NAME}_NEEDS_REFRESH "${needs_refresh}" PARENT_SCOPE)

    unset(FETCHCONTENT_FULLY_DISCONNECTED)
endfunction()
