FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libeigen3-dev \
        libnlopt-cxx-dev \
        liborocos-kdl-dev \
        ninja-build \
        pkg-config \
        python3 \
        python3-numpy \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .
RUN python3 scripts/export_g1_chains.py \
        robot/g1_body29_hand14.urdf \
        robot/chains \
    && cmake \
        -S native \
        -B native/build \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build native/build \
    && ctest \
        --test-dir native/build \
        --output-on-failure

ENV PYTHONPATH=/workspace/src
CMD ["python3", "-m", "pytest", "-q", "tests/traditional_grasp"]
