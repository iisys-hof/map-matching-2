# Build and run from project directory:
# docker build -t map_matching_2-clang-build -f docker/build/clang/18/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-clang-build

FROM debian:12 AS build

ARG DEBIAN_FRONTEND="noninteractive"
# only use major LLVM version
ARG LLVM_VERSION=18
ARG CMAKE_VERSION=3.30.5

RUN apt-get update && apt-get dist-upgrade -y && \
    apt-get install --no-install-recommends -y \
        tzdata make lsb-release ca-certificates wget software-properties-common gnupg xz-utils libssl-dev && \
    # https://github.com/hof/bookworm-apt-add-repository-issue
    [ ! -e /etc/apt/sources.list ] && echo "#" | tee /etc/apt/sources.list && \
    # LLVM
    wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh $LLVM_VERSION all && \
    rm llvm.sh && \
    update-alternatives  --install /usr/bin/clang   clang   /usr/bin/clang-$LLVM_VERSION 100 \
                         --slave   /usr/bin/clang++ clang++ /usr/bin/clang++-$LLVM_VERSION \
                         --slave   /usr/bin/lldb    lldb    /usr/bin/lldb-$LLVM_VERSION && \
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
        gdb ninja-build valgrind \
        locales-all dos2unix rsync tar python3 python3-dev linux-perf time git && \
    rm -rf /var/lib/apt/lists/*
