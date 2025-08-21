ARG BASE_IMAGE=ubuntu:20.04
FROM $BASE_IMAGE

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get dist-upgrade -y && \
    if apt-cache show fuse3 >/dev/null 2>&1; then \
        apt-get install -y fuse3; \
    else \
        apt-get install -y fuse; \
    fi && \
    rm -rf /var/lib/apt/lists/*

VOLUME /app
