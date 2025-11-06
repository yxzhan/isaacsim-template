# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:humble-py3.10

#============= Setup Isaac-sim python env =============#
# To shorten the build time, isaacsim will be mounted when the container starts instead of being installed in the image.
ENV ISAACSIM_VERSION="4.5"
ENV ISAACSIM_PATH="/mnt/dev-tools/isaac-sim-${ISAACSIM_VERSION}"
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Install jupyter kernel of Isaac-sim python (3.11)
COPY --chown=${NB_USER}:users binder/isaacsim_python_wrapper.sh /home/jovyan/.local/bin/isaacsim_python_wrapper.sh
COPY --chown=${NB_USER}:users binder/kernel.json /opt/conda/share/jupyter/kernels/isaac-sim//kernel.json

# Install UV
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
#===========================================================#

# Install vscode extensions
RUN code-server --install-extension ms-python.python
RUN code-server --install-extension ms-toolsai.jupyter

# Setup a Isaacsim ROS ws
USER ${NB_USER}
WORKDIR ${HOME}
RUN git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces && \
    cd IsaacSim-ros_workspaces/humble_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    pip install empy==3.3.4 && \
    source ${ROS_PATH}/setup.bash && \
    colcon build  --symlink-install --parallel-workers 8 && \
    echo "source $HOME/IsaacSim-ros_workspaces/humble_ws/install/setup.bash" >> ${HOME}/.bashrc

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

# Set Jupyterlab default directory
WORKDIR ${REPO_DIR}
# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
