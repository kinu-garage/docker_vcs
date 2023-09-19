# What is `docker_vcs`?

`docker_vcs` works as a wrapper for `docker build`. The tool takes in the software hosted on repositories (in the form of `.repos` file. Explained later) and/or `Dockerfile`, then build software inside of a Docker container. Outputs a Docker image.

In the primarily intended usecases, `docker_vcs` takes in and out:
- INPUT: 1) `X.repos` file, 2) a base Docker image.
- OUTPUT: A Docker image.
   - In the built Docker image, for the software packages that are cloned from the repositories defined in `X.repos`, dependent software will be installed, and the cloned software get built and installed.

`X.repos` is a `yaml`-formatted file where repositories can be listed. [vcstool](https://github.com/dirk-thomas/vcstool) can clone those repositories by batch. It allows you a systematic approach to define your software's dependency that consists of (multiple) software that is/are not available as a binary installer format.

Operation to do these are relatively easily achieved by the combination of a few `docker` CLI commands on the host + `bash` CLI commands inside a Docker container. However, even such a set of commands can get tedious, and little things can be easily missed in repeated processes. `docker_vcs` aims to make those trivial but tedious tasks easier and less error-prone.

## Usecases

Primary common usecases (UC) are as follows. In all UCs, you get a Docker image in the end:

- UC-1. You have `X.repos` file where a list of repositories the software you want to build and install are defined. You know which Docker image to build your software upon (aka "base" Docker image). The default build process will be applied to build your software.
- UC-2. In addition to UC-1 (you have `X.repos` file), you also have `Dockerfile`, i.e. a custom sets of commands to execute in order to fulfill dependencies and/or build and/or install your software.
   - Build process needs to be defined in the `Dockerfile` you're passing.
- UC-3. Unlike UC-1, you already have on your host the source code of your software, and the source of the dependencies.
   - If you want to do this manually, it could be done by e.g.:
      ```
      echo "On the host"
      export DOCKERIMG=docker pull ros:humble-ros-base
      export PATH_LOCAL_WS=/home/your_user/your_local_repo
      docker --network host --volume $PATH_LOCAL_WS:/workspace/src -- $DOCKERIMG bash
      
      echo "Then in a Docker container"
      source /opt/ros/$ROSDISTRO/setup.bash
      cd /workspace/src
      apt update && rosdep update && rosdep install --from-paths src --ignore-src -r -y
      colcon build
      
      echo "Back to host while the Docker container above is still alive"
      docker commit xx yy
      ```
      This will built and install your software and the dependencies you supplied the source code for. Also the dependencies of which binary installer is available are also installed.
- UC-4. Adding to UC-3, you also have a custom build and/or install process defined in a `Dockerfile`.
- UC-5. Similar to UC-4, you have `deps.repos` file where some/all of the source repositories defined.

### Supported combinations of input items
| Usecase ID | Dependent repos defined in `.repos` file? | Custom build/install process needed? | Source available on the host?
| -- | -- | -- | --
| UC-1 | Yes | No | No
| UC-2 | Yes | Yes | No
| UC-3 | No | No | Yes
| UC-4 | No | Yes | Yes
| UC-5 | Yes | Yes | Yes

### Advantages of `docker_vcs` in the mentioned UCs
Comparing with manual approach, `docker_vcs` provides the following (but not limited to) advantages:
- UC-1
   - Good when you aren't as proficient in commands to build your software that depends on multiple external software, and package it as a Docker image.
   - Concise operation for lengthy and error-prone operations.
   - Good on a CI host e.g. online dev repository.
- UC-2:
   - Good on a CI host e.g. online dev repository where you need custom steps in installation and/or building, e.g.
      - Dependency has to be "manually" installed.
         - `rosdep` cannot install those packages, because the packages your package depends on [are not available in `rosdep` database](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#what-if-my-library-isn-t-in-rosdistro).
         - Custom `apt` source (i.e. `/etc/apt/sources.list.d/YOUR.list` in Debian/Ubuntu), key server, are needed.
      - You need a special library where installation steps are already defined in a custom executable and you need to execute it.

# Usage
## First time users
As of 2023/09 `docker_vcs` is available in `pip`.

EoF
