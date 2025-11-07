# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:jazzy-py3.12

#============= Setup Isaac-sim python env =============#
# To shorten the build time, isaacsim will be mounted when the container starts instead of being installed in the image.
ENV ISAACSIM_VERSION="5.1"
ENV ISAACSIM_PATH="/mnt/dev-tools/isaac-sim-${ISAACSIM_VERSION}"
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Setting Up Isaacsim ROS workspace
ENV ISAAC_WS=${HOME}/isaac_ws
RUN cd /tmp && \
    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git && \
    mv IsaacSim-ros_workspaces/jazzy_ws $ISAAC_WS && \
    rm -rf IsaacSim-ros_workspaces && \
    cd $ISAAC_WS/src/moveit && \
    git clone https://github.com/ayushgnv/moveit_resources.git -b isaac_moveit_tutorial_jazzy && \
    cd $ISAAC_WS && \
    rosdep install -i --from-path src --rosdistro jazzy -y && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --parallel-workers 4

# Install jupyter kernel of Isaac-sim python (3.11)
COPY --chown=${NB_USER}:users binder/isaacsim_python_wrapper.sh /home/jovyan/.local/bin/isaacsim_python_wrapper.sh
COPY --chown=${NB_USER}:users binder/kernel.json /opt/conda/share/jupyter/kernels/isaac-sim//kernel.json

# Install UV
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
#===========================================================#

# Install vscode extensions
RUN code-server --install-extension ms-python.python
RUN code-server --install-extension ms-toolsai.jupyter

# Install python packages
USER ${NB_USER}
COPY --chown=${NB_USER}:users binder/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Copy repo content to the image
USER ${NB_USER}
ENV REPO_DIR=${HOME}/Documents/isaacsim-template
RUN mkdir -p ${REPO_DIR}
COPY --chown=${NB_USER}:users . ${REPO_DIR}/
RUN git config --global --add safe.directory ${REPO_DIR}
RUN ln -s $ISAAC_WS $PWD/isaac_ws

# Set Jupyterlab default directory
WORKDIR ${REPO_DIR}
# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
