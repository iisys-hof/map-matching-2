# Build and run from project directory:
# docker build -t map_matching_2-gcc-trunk-build -f docker/build/gcc/trunk/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-gcc-trunk-build

FROM debian:12 AS build

ARG DEBIAN_FRONTEND="noninteractive"
ARG GCC_VERSION=15
ARG CMAKE_VERSION=3.30.5

RUN apt-get update && apt-get dist-upgrade -y && \
    apt-get install --no-install-recommends -y \
        tzdata make wget file xz-utils libssl-dev && \
    apt-get install -y \
        build-essential dpkg-dev flex bison texinfo libgmp3-dev libmpfr-dev libmpc-dev libisl-dev && \
    apt-get install -y \
        python3-dev libipt-dev libexpat1-dev liblzma-dev libbabeltrace-dev && \
    # GCC
    mkdir /opt/build && cd /opt/build && \
    wget -r -l 1 -nH -nd -np -A '*.tar.xz' "https://gcc.gnu.org/pub/gcc/snapshots/LATEST-${GCC_VERSION}/" && \
    GCC_FILE=$(ls -1 gcc-*.tar.xz) && \
    tar -xf "$GCC_FILE" && \
    GCC_PATH=$(basename "$GCC_FILE" '.tar.xz') && \
    cd "$GCC_PATH" && mkdir objdir && cd objdir && \
    ../configure --enable-languages=c,c++ --disable-multilib --build=x86_64-pc-linux-gnu && \
    make -j "$(nproc)" && \
    make install-strip && \
    cd / && rm -rf /opt/build && \
    echo '/usr/local/lib64' > /etc/ld.so.conf.d/local-lib.conf && \
    echo '/usr/local/lib' >> /etc/ld.so.conf.d/local-lib.conf && \
    ldconfig -v && \
    dpkg-divert --divert /usr/bin/gcc.orig --rename /usr/bin/gcc && \
    dpkg-divert --divert /usr/bin/g++.orig --rename /usr/bin/g++ && \
    update-alternatives --install /usr/bin/cc cc /usr/local/bin/gcc 999 && \
    # CMake
    wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz" && \
    tar -xzf "cmake-${CMAKE_VERSION}.tar.gz" && \
    rm -f "cmake-${CMAKE_VERSION}.tar.gz" && \
    cd "cmake-${CMAKE_VERSION}" && \
    ./bootstrap --parallel=$(nproc) && make -j $(nproc) && make install && \
    cd .. && rm -rf "cmake-${CMAKE_VERSION}" && \
    update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 100 && \
    # gdb
    mkdir /opt/build && cd /opt/build && \
    wget "https://gcc.gnu.org/pub/gdb/snapshots/current/gdb.tar.xz" && \
    GDB_FILE=$(ls -1 gdb-*.tar.xz) && \
    tar -xf "$GDB_FILE" && \
    GDB_PATH=$(basename "$GDB_FILE" '.tar.xz') && \
    cd "$GDB_PATH" && mkdir objdir && cd objdir && \
    ../configure --build=x86_64-linux-gnu --host=x86_64-linux-gnu --with-expat --with-lzma --with-babeltrace --with-python=python3 && \
    make -j "$(nproc)" && make install && \
    cd / && rm -rf /opt/build && ldconfig -v && \
    update-alternatives --install /usr/bin/gdb gdb /usr/local/bin/gdb 100 && \
    # debug tools
    apt-get install --no-install-recommends -y \
        clang ninja-build autoconf automake libtool valgrind \
        locales-all dos2unix rsync tar python3 python3-dev linux-perf time git && \
    rm -rf /var/lib/apt/lists/*
