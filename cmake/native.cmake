include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-march=native" _march_native)

set(_CXX_FLAGS)
if(_march_native)
    set(_CXX_FLAGS "-march=native" CACHE STRING "native compile flags" FORCE)
    message(STATUS "_CXX_FLAGS set to ${_CXX_FLAGS}")
endif()
