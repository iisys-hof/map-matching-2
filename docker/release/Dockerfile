# Build and run from project directory:
# docker build -t map_matching_2 -f docker/release/Dockerfile .
# docker run --rm -v .:/tmp/map-matching-2 -u $(id -u):$(id -g) map_matching_2

FROM map_matching_2-gcc-ubuntu-2204-build AS build

ARG DEBIAN_FRONTEND="noninteractive"

WORKDIR /app

COPY LICENSE.md .
COPY CMakeLists.txt .
COPY external external
COPY src src

RUN cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF -DEXPLICIT_TEMPLATES=OFF -B build && \
    cmake --build build --parallel $(nproc) --target install

FROM ubuntu:20.04 AS run

COPY --from=build /app/run/gcc/release /app/run

VOLUME /app/data

WORKDIR /app

ENTRYPOINT ["/app/run/bin/map_matching_2"]
