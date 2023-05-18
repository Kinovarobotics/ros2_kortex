# syntax=docker/dockerfile:1.2
ARG ROS_DISTRO="rolling"
FROM osrf/ros:${ROS_DISTRO}-desktop-full AS upstream
# Restate for later use
ARG ROS_DISTRO
ARG REPO

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install build dependencies
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        build-essential \
        ccache \
        cmake \
	    lcov \
        lld \
        python3-colcon-common-extensions \
	    python3-colcon-lcov-result \
        python3-colcon-coveragepy-result \
        python3-colcon-mixin \
    && rm -rf /var/lib/apt/lists/*

# build source dependencies
WORKDIR /opt/upstream
COPY ros2_kortex.${ROS_DISTRO}.repos .
COPY ros2_kortex-not-released.${ROS_DISTRO}.repos . 
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    mkdir src \
    # && vcs import src < ros2_kortex.${ROS_DISTRO}.repos \
    && vcs import src < ros2_kortex-not-released.${ROS_DISTRO}.repos \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update && apt-get update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --mixin release lld \
    && rm -rf build log src ros2_kortex.${ROS_DISTRO}.repos

# copy source to install repo dependencies
WORKDIR /opt/ws
COPY . ./src/${REPO}

# install repo dependencies
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    . /opt/upstream/install/setup.sh \
    && rosdep update && apt-get update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM upstream AS development

ARG UID
ARG GID
ARG USER

# fail build if args are missing
# hadolint ignore=SC2028
RUN if [ -z "$UID" ]; then echo '\nERROR: UID not set. Run \n\n \texport UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$GID" ]; then echo '\nERROR: GID not set. Run \n\n \texport GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        clang-14 \
        clang-format-14 \
        clang-tidy-14 \
        git \
	    openssh-client \
        python3-pip \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

# install developer tools
RUN python3 -m pip install --no-cache-dir \
    pre-commit

# Setup user home directory
# --no-log-init helps with excessively long UIDs
RUN groupadd --gid ${GID} ${USER} \
    && useradd --no-log-init --uid ${UID} --gid ${GID} -m ${USER} --groups sudo \
    && echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER} \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.profile \
    && touch /home/${USER}/.bashrc \
    && chown -R ${UID}:${GID} /home/${USER}

USER ${USER}
ENV SHELL /bin/bash
ENTRYPOINT []

# Setup mixin
WORKDIR /home/${USER}/ws
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default
