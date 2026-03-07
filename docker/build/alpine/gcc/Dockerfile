# Build and run from project directory:
# docker build -tmap_matching_2-alpine-gcc-build -f docker/build/alpine/gcc/Dockerfile .
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-alpine-gcc-build

FROM alpine:edge AS build

RUN apk update && \
    apk add linux-headers build-base cmake ninja gdb git

ENV CC=gcc
ENV CXX=g++
ENV CMAKE_GENERATOR=Ninja
