# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:jazzy-py3.12

USER ${NB_USER}
#============= Setup Isaac-sim python env =============#
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


# Install jupyter kernel of Isaac-sim python (3.11)
COPY --chown=${NB_USER}:users binder/isaacsim_python_wrapper.sh /home/jovyan/.local/bin/isaacsim_python_wrapper.sh
COPY --chown=${NB_USER}:users binder/kernel.json /opt/conda/share/jupyter/kernels/isaac-sim//kernel.json

# Install UV
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
#===========================================================#

# Install vscode extensions
RUN code-server --install-extension ms-python.python
RUN code-server --install-extension ms-toolsai.jupyter
RUN code-server --install-extension continue.continue

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

# Set Jupyterlab default directory
WORKDIR ${REPO_DIR}
# Set VSCODE default directory
ENV CODE_WORKING_DIRECTORY=${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
