# syntax=docker/dockerfile:1.7

ARG PIXI_IMAGE=ghcr.io/prefix-dev/pixi:0.84.1-noble-cuda-12.9.1
ARG BASE_IMAGE=nvidia/cuda:12.9.1-runtime-ubuntu24.04
ARG PIXI_ENV=jazzy

FROM ${PIXI_IMAGE} AS build

ARG PIXI_ENV
WORKDIR /opt/contact_graspnet_ros

COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e ${PIXI_ENV}
RUN pixi shell-hook -e ${PIXI_ENV} -s bash > /shell-hook

COPY . .
RUN pixi run --locked -e ${PIXI_ENV} build

FROM ${BASE_IMAGE} AS production

ARG PIXI_ENV
WORKDIR /opt/contact_graspnet_ros

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# Keep the same absolute prefix as the build stage: pixi's shell-hook embeds it.
COPY --from=build /opt/contact_graspnet_ros/.pixi/envs/${PIXI_ENV} /opt/contact_graspnet_ros/.pixi/envs/${PIXI_ENV}
COPY --from=build /opt/contact_graspnet_ros/install /opt/contact_graspnet_ros/install
COPY --from=build /opt/contact_graspnet_ros/src /opt/contact_graspnet_ros/src
COPY --from=build /opt/contact_graspnet_ros/third_party /opt/contact_graspnet_ros/third_party
COPY --from=build /opt/contact_graspnet_ros/config /opt/contact_graspnet_ros/config
COPY --from=build /opt/contact_graspnet_ros/scripts /opt/contact_graspnet_ros/scripts
COPY --from=build /opt/contact_graspnet_ros/pixi.toml /opt/contact_graspnet_ros/pixi.toml
COPY --from=build /shell-hook /shell-hook

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV CONTACT_GRASPNET_ROOT=/opt/contact_graspnet_ros \
    GRASPNET_CONFIG_DIR=/opt/contact_graspnet_ros/config \
    GRASPNET_CHECKPOINT_DIR=/opt/contact_graspnet_ros/checkpoints \
    ROS_DISTRO=${PIXI_ENV} \
    PYTHONUNBUFFERED=1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "contact_graspnet_ros", "graspnet.launch.py"]
