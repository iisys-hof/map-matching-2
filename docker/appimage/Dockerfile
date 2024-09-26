# Build and run from project directory:
# DOCKER_BUILDKIT=1 docker build --privileged -t map_matching_2-appimage -f docker/appimage/Dockerfile --output . .
# docker run --device /dev/fuse --cap-add SYS_ADMIN --security-opt apparmor:unconfined -it --rm map_matching_2-appimage
# docker run -u $(id -u):$(id -g) -v .:/tmp/map-matching-2 -it --rm map_matching_2-appimage

FROM map_matching_2-gcc-ubuntu-2004-build AS build

ARG DEBIAN_FRONTEND="noninteractive"

WORKDIR /app

COPY LICENSE.md .
COPY CMakeLists.txt .
COPY external external
COPY src src

RUN cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=OFF -DEXPLICIT_TEMPLATES=OFF -DCMAKE_INSTALL_PREFIX=/usr -B build && \
    DESTDIR=/app/AppDir cmake --build build --parallel $(nproc) --target install

FROM scratch AS export

COPY --from=build /app/AppDir/ /AppDir
