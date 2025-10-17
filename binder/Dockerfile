# Dockerfile of the base image: https://github.com/IntEL4CoRo/jupyter-ros2
FROM intel4coro/jupyter-ros2:jazzy-py3.12

#============= Setup Isaac-sim python env =============#
# To shorten the build time, isaacsim will be mounted when the container starts instead of being installed in the image.
ENV ISAACSIM_VERSION="5.0"
ENV ISAACSIM_PATH="/mnt/dev-tools/isaac-sim-${ISAACSIM_VERSION}"
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Install jupyter kernel of Isaac-sim python (3.11)
COPY --chown=${NB_USER}:users binder/kernel.json /opt/conda/share/jupyter/kernels/isaac-sim/kernel.json
#===========================================================#

# Copy content to the image
USER ${NB_USER}
ENV REPO_DIR=${HOME}/Documents/isaacsim-template
RUN mkdir -p ${REPO_DIR}
WORKDIR ${REPO_DIR}
COPY --chown=${NB_USER}:users . ${REPO_DIR}/
RUN git config --global --add safe.directory ${REPO_DIR}

# The entrypoint of the docker image
COPY --chown=${NB_USER}:users binder/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
