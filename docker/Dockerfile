FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       cmake \
       pkg-config \
       libopencv-dev \
       libceres-dev \
       libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/ecal
COPY . .

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DECAL_STANDALONE=ON \
    && cmake --build build -j"$(nproc)"

FROM ubuntu:22.04 AS runtime-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       libopencv-dev \
       libceres-dev \
       libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

FROM runtime-base AS checkerboard
COPY --from=builder /workspace/ecal/build/ecal_checkerboard_calibrate /usr/local/bin/ecal_checkerboard_calibrate
COPY --from=builder /workspace/ecal/docker/checkerboard.docker.yaml /app/config/checkerboard.docker.yaml
ENTRYPOINT ["/usr/local/bin/ecal_checkerboard_calibrate"]
CMD ["/app/config/checkerboard.docker.yaml"]
