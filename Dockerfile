FROM ubuntu:20.04 AS build

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get dist-upgrade -y
RUN apt-get install -y build-essential cmake git
RUN apt-get install -y zlib1g-dev libbz2-dev libexpat1-dev libproj-dev

WORKDIR /app

COPY CMakeLists.txt .
COPY cmake cmake
COPY external external
COPY src src

RUN cmake -DCMAKE_BUILD_TYPE=Release -DINSTALL_ALL_SHARED_LIBS=ON -DENABLE_TESTS=OFF -B build
RUN cmake --build build --parallel $(nproc) --target install

FROM ubuntu:20.04 AS run

COPY --from=build /app/run /app/run

VOLUME /app/data

WORKDIR /app/run/bin

ENTRYPOINT /bin/bash
