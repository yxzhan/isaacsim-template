# isaacsim-template

[![Binder](https://binder.intel4coro.de/badge_logo.svg)](https://binder.dev.intel4coro.de/v2/gh/yxzhan/isaacsim-template/main?urlpath=lab%2Ftree%2Fexamples%2Flauncher.ipynb)

This is a template repo for running modern robot simulators (such as Isaac Sim and Unreal Engine) on the GPU-enabled AIRCOR VRB cloud server.

https://github.com/user-attachments/assets/1b3cd8d2-048c-43c3-9e97-31fb69fb479b

> ### Access Requirements:
> 
> Currently, the GPU-enabled server is only accessible within the Uni-Bremen network. You need to be connected through one of the following:
>
> - Office LAN network
> - Eduroam WiFi on campus
> - University VPN (vpn.uni-bremen.de)

**If you cannot access the server, you can also run locally on your PC. See the [Local Development](#local-development) section for instructions.**

## Quick start

1. Open the following link in a new browser tab to launch a lab instance:

    https://binder.dev.intel4coro.de/v2/gh/yxzhan/isaacsim-template/main?urlpath=lab/tree/examples/launcher.ipynb

1. The notebook `launcher.ipynb` is a UI interface of [ipywidgets](https://github.com/jupyter-widgets/ipywidgets) for quickly running the demos. Follow the notebook instructions to initialize the UI. If you see the low memory warning message, it indicates that someone else is currently using the GPU resources. Please wait until it is free before trying again.

1. Since Isaac Sim only supports Python 3.11 while the default Python environment (aligned with ROS Jazzy) is version 3.12. To run Isaac Sim code within the notebook, you need to set the kernel to "Isaac Sim Python 3.11". In the same directory, there is an example notebook named [apartment.ipynb](./examples/apartment.ipynb), which provides a simple step-by-step  tutorial covering some fundamental usage scenarios.

    ![](./docs/tutorial.png)

1. Alternatively, you can open the code with `VSCode` and configure the Python interpreter to `/isaac-sim/python.sh`.

    ![vscode](./docs/vscode.gif)

1. If you are running programs that utilize GPU resources, such as Isaac Sim or Unreal Engine, please remember to manually terminate the processes afterward, as GPU resources are highly limited. Best to manually shut down the entire lab instance in menu `File > Shutdown`.

    ![](./docs/shutdown.jpg)

> Note: Currently, only Vulkan-based programs can utilize GPU rendering, while OpenGL-based applications (such as Rviz, PyBullet, Mujoco, Gazebo, etc.) are rendering by CPU.

## Upgrade existing VRB labs

The existing VRB labs can also run directly on the GPU server by simply changing the launch URL from "https://binder.XXX" to "https://binder.dev.XXX". However, to run Isaac Sim, the Docker image needs to be upgraded to at least Ubuntu 22.04 and requires installing a VNC desktop, the recommended way is to update the base Docker image to `intel4coro/jupyter-ros2:jazzy-py3.12`.

## Create a new VRB lab from this template

1. Login to Github.

1. Use this template repository to create a new repository or fork it. Forking will make it easier to sync with future updates.

    ![](./docs/create-repo.png)

1. Clone your git repo, add your notebooks, Python code, USD files to the repo.

1. Modify the [binder/requirements.txt](binder/requirements.txt) to install additional Python packages.

1. Modify the [binder/Dockerfile](binder/Dockerfile) if your project needs additional APT packages.

    Examples:

    ```Dockerfile
    # Install APT packages (switch to root user)
    USER root
    RUN apt update
    RUN apt install -y ffmpeg
    # Switch back to the non-root user to avoid file permission issues.
    USER ${NB_USER}
    ```

1. Launch your VRB lab instance, replacing the placeholder content inside the curly braces `{}` with your actual information, and open it in a web browser.

    ```
    https://binder.dev.intel4coro.de/v2/gh/{YOUR_GITHUB_USER_NAME}/{YOUR_REPO_NAME}/main?urlpath=lab/tree/{PATH_TO_NOTEBOOK}
    ```

    The first time it is launched, it will take some time to build the Docker image.

<!-- ## Repository Structure

```
.
├── binder/           # Defines the container environment
├── examples/         # Jupyter notebook and python examples
└── usd/              # Example USD file (iai_apartment)
```

## Lab Runtime Structure

The runtime environment can be divided into three layers:

- The top layer (GitHub repo) is fully controllable by users.
- The middle shared directory layer allows restricted user access.
- The bottom GPU toolkit layer can only be managed by the background cloud system.

![](./docs/runtime-arch.png) -->

## Local Development

### System Requirements

- Ubuntu 22.04 or higher
- 16GB RAM
- NVIDIA RTX 2070 or higher
- At least 50GB free disk space

> To check your GPU model and verify that NVIDIA drivers are installed, run `nvidia-smi` in the terminal.

### Install Docker and NVIDIA Container Toolkit

Check if installed, otherwise follow the docs below:

Docker:

```
sudo docker --version

# Output
Docker version 28.2.2, build 28.2.2-0ubuntu1~22.04.1
```

NVIDIA Container Toolkit:

```
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi

# Output
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.86.10    Driver Version: 535.86.10    CUDA Version: 12.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
| N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|  No running processes found                                                 |
+-----------------------------------------------------------------------------+
```

#### Installation Guide:

- Docker: https://docs.docker.com/engine/install/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

### Option A: Run without building docker image

1. Create an empty directory for isaacsim shader cache (e.g., `mkdir ~/isaac_cache` and `cd ~/isaac_cache`).

1. Execute under the cache directory:

    ```bash
    sudo docker run --rm --gpus all -it \
    --cpus=8 \
    --memory=16g \
    --user root \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env ACCEPT_EULA="YES" \
    --env PRIVACY_CONSENT="YES" \
    --env OMNI_KIT_ACCEPT_EULA="YES" \
    --env OMNI_KIT_ALLOW_ROOT=1 \
    -v /usr/share/vulkan/icd.d/:/etc/vulkan/icd.d \
    -v ${PWD}:/isaac-sim/kit/cache \
    -p 8888:8888 \
    intel4coro/yxzhan-2disaacsim-2dtemplate-b9ff8d:bdb959b02833edc0c24b8649c2990e2148cec286 \
    jupyter lab --allow-root --NotebookApp.token='' --no-browser --ip=0.0.0.0
    ```

    Command parameters explanation:

    - `--rm`: Automatically remove the container when it exits.
    - `--gpus all`: Enable all available GPUs for the container.
    - `--cpus=8`: Limit the container to use up to 8 CPU cores.
    - `--memory=16g`: Limit the container's memory usage to 16GB.
    - `--user root`: Run as root in docker container.
    - `--env NVIDIA_DRIVER_CAPABILITIES=all`: Set NVIDIA driver capabilities to all.
    - `--env ACCEPT_EULA="YES"`: Accept the End User License Agreement.
    - `--env PRIVACY_CONSENT="YES"`: Consent to privacy terms.
    - `--env OMNI_KIT_ACCEPT_EULA="YES"`: Accept Omniverse Kit EULA.
    - `--env OMNI_KIT_ALLOW_ROOT=1`: Allow running as root in Omniverse Kit.
    - `-v /usr/share/vulkan/icd.d/:/etc/vulkan/icd.d`: Mount host machine Vulkan ICD directory.
    - `-v ${PWD}:/isaac-sim/kit/cache`: Mount current directory to Isaac Sim cache directory.
    - `-p 8888:8888`: Map port 8888 on host to port 8888 in container.

1. Open the demo notebook: http://127.0.0.1:8888/lab/tree/examples/launcher.ipynb

    > Note: On the first launch, Isaac Sim will compile shaders based on your current GPU. This process may take over 10 minutes and will heavily utilize your system's CPU, and memory resources. This is expected behavior — please be patient and do not interrupt the process. Subsequent launches will be significantly faster as the compiled shaders are cached.

1. To stop and remove the container, simply press `Ctrl+C` in the terminal.

### Option B: Build docker image

- Clone this repo.

- Build docker image (Under repo directory):

  ```bash
  sudo docker compose -f ./binder/docker-compose.yml up --build
  ```

- To stop and remove container:

  ```bash
  sudo docker compose -f ./binder/docker-compose.yml down
  ```
