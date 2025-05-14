# Build and run from project directory:
# docker build -t map_matching_2-gcc-hub-build -f docker/build/gcc/hub/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-gcc-docker-build

FROM gcc:15 AS build

ARG DEBIAN_FRONTEND="noninteractive"
ARG CMAKE_VERSION=3.31.6

RUN apt-get update && apt-get dist-upgrade -y && \
    apt-get install --no-install-recommends -y \
        tzdata make wget file xz-utils libssl-dev && \
    apt-get install -y \
        build-essential && \
    # CMake
    wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz" && \
    tar -xzf "cmake-${CMAKE_VERSION}.tar.gz" && \
    rm -f "cmake-${CMAKE_VERSION}.tar.gz" && \
    cd "cmake-${CMAKE_VERSION}" && \
    ./bootstrap --parallel=$(nproc) && make -j $(nproc) && make install && \
    cd .. && rm -rf "cmake-${CMAKE_VERSION}" && \
    update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 100 && \
    # debug tools
    apt-get install --no-install-recommends -y \
        gdb clang ninja-build autoconf automake libtool valgrind \
        locales-all dos2unix rsync tar python3 python3-dev linux-perf time git && \
    rm -rf /var/lib/apt/lists/*
