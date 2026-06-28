FROM ros:humble-ros-base-jammy AS cacher

WORKDIR /ws/src

COPY . lighthouse_ros_ros2_stack/

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN mkdir -p /tmp/ws/src \
 && find ./ -name "package.xml" -print0 | xargs -0 cp --parents -t /tmp/ws/src \
 && find ./ -name "COLCON_IGNORE" -print0 | xargs -0 cp --parents -t /tmp/ws/src \
 || true

FROM ros:humble-ros-base-jammy AS main_development_image

ARG USER=developer
ARG GROUP=developer

ENV DEBIAN_FRONTEND noninteractive

# DL3008: Versions not pinned in dev image to get latest packages
# hadolint ignore=DL3008
RUN apt-get update \
 && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gdb \
    git \
    python3-pip \
    mc \
    socat \
    tmux \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pre-commit==2.20.0 spatialmath-python==1.1.9

RUN addgroup --gid 1000 $GROUP \
 && adduser --uid 1000 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
 && adduser $USER sudo \
 && adduser $USER dialout \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

USER $USER:$GROUP

ENV USER_WS /home/$USER/ws

WORKDIR $USER_WS

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
 && colcon mixin update default

COPY --chown=$USER:$GROUP docker/files/colcon_defaults.yaml /home/$USER/.colcon/defaults.yaml

#
# Install project dependencies (from the package.xml files isolated in the cacher stage).

COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# DL3004: sudo needed since we're running as non-root user
# hadolint ignore=DL3004
RUN sudo apt-get update \
 && . /opt/ros/humble/setup.sh \
 && rosdep update \
 && rosdep install -i -y --from-path "$USER_WS/src" \
 && sudo rm -rf /var/lib/apt/lists/* \
 && echo Done

COPY --chown=root:root docker/files/fixuid_config.yml /etc/fixuid/config.yml

# DL3003: cd used for simplicity, DL3004: sudo needed since we're running as non-root user
# hadolint ignore=DL3003,DL3004
RUN ARCH=$(uname -m) && if [ "$ARCH" = "aarch64" ]; then FIXUID_ARCH="arm64"; else FIXUID_ARCH="amd64"; fi \
  && curl -SsL "https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-${FIXUID_ARCH}.tar.gz" | sudo tar -C /usr/local/bin -xzf - \
  && sudo chmod 4755 /usr/local/bin/fixuid \
  && cd /etc/fixuid \
  && sudo sed -i "s/_USER_/$USER/" config.yml \
  && sudo sed -i "s/_GROUP_/$GROUP/" config.yml \
  && echo "Done installing fixuid for ${FIXUID_ARCH}"

#
# Final setup of the workspace

RUN mkdir -p $USER_WS/persistence && chmod 777 $USER_WS/persistence

USER root
COPY docker/files/ros_entrypoint.sh /ros_entrypoint.sh

USER $USER:$GROUP

ENV WITHIN_DEV 1

CMD        ["bash"]
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh"]
