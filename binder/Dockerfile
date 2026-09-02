# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:jazzy-py3.12

USER ${NB_USER}
#============= Install IsaacSim =============#
# To shorten the build time, isaacsim will be mounted when the container starts instead of being installed in the image.
ENV ISAACSIM_VERSION="5.1"
ENV ISAACSIM_PATH="/isaac-sim"
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Download IsaacSim
RUN cd /tmp && \
    wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-${ISAACSIM_VERSION}.0-linux-x86_64.zip && \
    sudo unzip /tmp/isaac-sim-standalone-${ISAACSIM_VERSION}.0-linux-x86_64.zip -d ${ISAACSIM_PATH} && \
    sudo chown ${NB_USER}:users -R ${ISAACSIM_PATH} && \
    rm /tmp/isaac-sim-standalone-${ISAACSIM_VERSION}.0-linux-x86_64.zip

# Install extra packages to isaacsim python env
RUN $ISAACSIM_PYTHON_EXE -m pip install \
        tqdm \
        sidecar \
        pandas \
        jupyter \
        ipywidgets \
        matplotlib

# Install jupyter kernel of Isaac-sim python (3.11)
COPY --chown=${NB_USER}:users binder/isaacsim_python_wrapper.sh /home/jovyan/.local/bin/isaacsim_python_wrapper.sh
COPY --chown=${NB_USER}:users binder/kernel.json /opt/conda/share/jupyter/kernels/isaac-sim//kernel.json

# Install UV
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
#===========================================================#

# Install vscode extensions
RUN mamba update -y conda-forge::code-server
RUN code-server --install-extension mhutchie.git-graph
RUN code-server --install-extension lichenxi.sysmonitor
RUN curl -LO https://github.com/yxzhan/vscode_remote_desktop/releases/download/v0.1.0/vscode-remote-desktop-0.1.0.vsix && \
    code-server --install-extension vscode-remote-desktop-0.1.0.vsix && \
    rm vscode-remote-desktop-0.1.0.vsix

# Install Claude code
RUN curl -fsSL https://claude.ai/install.sh | bash

#============= Install ROS 2 Nav2 + SLAM (Jazzy) =============#
# Navigation stack used by examples/apartment.ipynb (SLAM + Nav2 for Stretch).
# The base image pins the ROS core at an older snapshot than these Nav2 builds,
# so fastcdr must be upgraded too -- otherwise the C++ nodes (controller_server,
# slam_toolbox) die with `undefined symbol: ..._ZN8eprosima7fastcdr3Cdr9serializeEj`
# (Fast-CDR ABI mismatch).
#
# packages.ros.org is rolling (only the latest build of each package is kept), so
# pulling ros-jazzy-fastcdr alone gets a fastcdr that is NEWER than the base
# image's fastrtps / rmw_fastrtps / rosidl typesupport. That skew makes EVERY node
# creation abort inside Fast-DDS's XTypes type-cache with
#   eprosima::fastcdr::exception::BadParamException: This member is not been selected
# (see examples/launcher.ipynb -> CmdVelPublisher). So upgrade the whole Fast-DDS
# serialization stack together to keep fastcdr and fastrtps consistent.
RUN sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-jazzy-fastcdr \
        ros-jazzy-fastrtps \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-jazzy-rmw-fastrtps-shared-cpp \
        ros-jazzy-rosidl-dynamic-typesupport \
        ros-jazzy-rosidl-dynamic-typesupport-fastrtps \
        ros-jazzy-rosidl-typesupport-fastrtps-cpp \
        ros-jazzy-rosidl-typesupport-fastrtps-c \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox && \
    sudo rm -rf /var/lib/apt/lists/*
#============================================================#

RUN /isaac-sim/python.sh -m pip install h5py imageio-ffmpeg

# Install python packages
USER ${NB_USER}
COPY --chown=${NB_USER}:users binder/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Copy repo content to the image
USER ${NB_USER}
ENV REPO_DIR=${HOME}/isaacsim-template
RUN mkdir -p ${REPO_DIR}
COPY --chown=${NB_USER}:users . ${REPO_DIR}/
RUN git config --global --add safe.directory ${REPO_DIR}

# Build the Nav2 ROS workspace (stretch_nav2: launch + params + rviz) so the
# apartment.ipynb "Launch SLAM + Nav2" cell can source it directly.
RUN cd ${REPO_DIR}/nav2_ws && \
    bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

USER root
RUN apt update && \
    apt install -y git-lfs && \
    git lfs install
# RUN cd ${REPO_DIR} && \
#     git clone https://github.com/zhiyuan-gao/kitchen_imitation_learning.git

# Set Jupyterlab default directory
WORKDIR ${REPO_DIR}
# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
