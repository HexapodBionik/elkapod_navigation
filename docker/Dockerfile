ARG BASE_IMAGE=ghcr.io/hexapodbionik/elkapod-ros-jazzy-base:latest

# --------------------------------------------------------------- manifests --
# Collect just the package.xml files. COPY --from hashes the content it copies,
# so the expensive rosdep layer below stays cached whenever only sources change.
FROM ${BASE_IMAGE} AS manifests
COPY . /tmp/repo
RUN mkdir -p /tmp/manifests && cd /tmp/repo && \
    find . -name package.xml -exec cp --parents {} /tmp/manifests/ \;

# -------------------------------------------------------------------- deps --
FROM ${BASE_IMAGE} AS deps
SHELL ["/bin/bash", "-lc"]
ARG REPO_NAME=elkapod_navigation
ARG ROSDEP_SKIP_KEYS=""
WORKDIR /ws

COPY repos.yaml /tmp/repos.yaml
RUN mkdir -p src && \
    vcs import --shallow src < /tmp/repos.yaml && \
    touch src/elevation_mapping_cupy/plane_segmentation/COLCON_IGNORE

# CuPy 14 requires numpy>=2, ROS Jazzy needs numpy 1.x.
RUN --mount=type=cache,target=/home/ubuntu/.cache/pip,uid=1000,gid=1000 \
    pip install --index-url https://download.pytorch.org/whl/cu126 torch && \
    pip install "numpy<2" "cupy-cuda12x[ctk]<14" simple-parsing

# CuPy 13 does not find the pip-installed CUDA libraries on its own.
ARG CUDA_PIP_LIBS=/ws/.venv/lib/python3.12/site-packages/nvidia
ENV LD_LIBRARY_PATH=${CUDA_PIP_LIBS}/cuda_nvrtc/lib:${CUDA_PIP_LIBS}/cuda_runtime/lib:${CUDA_PIP_LIBS}/cublas/lib:${CUDA_PIP_LIBS}/curand/lib:${CUDA_PIP_LIBS}/cusolver/lib:${CUDA_PIP_LIBS}/cusparse/lib:${CUDA_PIP_LIBS}/cufft/lib:${CUDA_PIP_LIBS}/nvjitlink/lib

COPY --chown=ubuntu:ubuntu --from=manifests /tmp/manifests /ws/src/${REPO_NAME}

# Pulls in nav2, rtabmap and rviz2 -- by far the slowest layer, hence the
# manifest-only staging above.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -y \
        --rosdistro "${ROS_DISTRO}" \
        ${ROSDEP_SKIP_KEYS:+--skip-keys "${ROSDEP_SKIP_KEYS}"}

RUN sudo install -d -o ubuntu -g ubuntu /elkapod_sim_ws/data

# ------------------------------------------------------------------- build --
FROM deps AS build
SHELL ["/bin/bash", "-lc"]
ARG REPO_NAME=elkapod_navigation
ARG COLCON_PACKAGES=""
WORKDIR /ws

COPY --chown=ubuntu:ubuntu . /ws/src/${REPO_NAME}

RUN --mount=type=cache,target=/home/ubuntu/.ccache,uid=1000,gid=1000 \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        ${COLCON_PACKAGES:+--packages-up-to ${COLCON_PACKAGES}}

COPY --chmod=755 docker/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
