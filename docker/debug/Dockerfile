# Build and run from project directory:
# docker build -t map_matching_2-debug -f  docker/debug/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-debug-build

FROM map_matching_2-gcc-build AS build

ARG DEBIAN_FRONTEND="noninteractive"

WORKDIR /app

RUN git clone https://github.com/royjacobson/externis.git
RUN git clone https://github.com/fmtlib/fmt.git
RUN git clone https://github.com/aras-p/ClangBuildAnalyzer.git

WORKDIR /app/fmt

RUN git checkout 10.2.1 && \
    cmake -DCMAKE_CXX_COMPILER=g++ -B build && \
    cmake --build build --parallel $(nproc) --target install

WORKDIR /app/externis

RUN cmake -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build && \
    cmake --build build --parallel $(nproc) --target install

WORKDIR /app/ClangBuildAnalyzer

RUN cmake -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build && \
    cmake --build build --parallel $(nproc) --target install

# Commands:
# FOLDER=/tmp/map-matching-2/cmake-build-debug-clang/src
#
# ./ClangBuildAnalyzer --start "$FOLDER"
# ./ClangBuildAnalyzer --stop  "$FOLDER" "$FOLDER/Capture.bin"
# ./ClangBuildAnalyzer --analyze "$FOLDER/Capture.bin"
# or
# ./ClangBuildAnalyzer --all "$FOLDER" "$FOLDER/Capture.bin"
