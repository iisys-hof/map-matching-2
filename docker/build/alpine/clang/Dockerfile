# Build and run from project directory:
# docker build -t map_matching_2-alpine-clang-build -f docker/build/alpine/clang/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-alpine-clang-build

FROM alpine:edge AS build

RUN apk update && \
    apk add linux-headers build-base cmake ninja gdb git clang

ENV CC=clang
ENV CXX=clang++
ENV CMAKE_GENERATOR=Ninja
