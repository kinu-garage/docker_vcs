ARG BASE_DIMG
FROM ${BASE_DIMG}

SHELL ["/bin/bash", "-c"]

ARG ENTRY_POINT
# Re-defining ROSDISTRO, as 'ARG's need to be defined after FROM, except the one meant to be passed to FROM https://stackoverflow.com/a/63086565/577001
ARG ROSDISTRO="noetic"
ARG UBUNTU_DISTRO=focal
ARG path_repos
ARG workspace_in_container

RUN echo "DEBUG: ${ROSDISTRO}"

# Hint to dpkg and apt that packages should be installed without asking for human intervention
ENV DEBIAN_FRONTEND noninteractive

# Need these before sources can be set up
RUN apt-get update \
 && apt-get install -y gnupg2 \
 # setup keys and sources for official Gazebo debian packages
 && apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 \
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_DISTRO main" > /etc/apt/sources.list.d/gazebo-latest.list \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# install OS tools
RUN apt-get update && apt-get install -q -y \
    bash-completion \
    git \
    locales \
    psmisc \
    python3-rosdep \
    python3-vcstool \
    wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Expose port used to communicate with gzserver
EXPOSE 11345

# Stuff for nvidia-docker
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Set encoding to use unicode characters
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Create a new user called ariac. Note: we don't add them to the sudo group
ENV USERNAME ariac
ARG USERID=1000
RUN adduser -u $USERID --gecos "Development User" --disabled-password $USERNAME \
    && echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME

# Get gazebo models early since it is big
##RUN wget -P /tmp/ https://github.com/osrf/gazebo_models/archive/master.zip \
## && mkdir -p $HOME/.gazebo/models \
## && tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1 \
## && rm /tmp/default.tar.gz

#Download and compile ARIAC 2020
RUN mkdir -p $workspace_in_container/src && wget https://raw.githubusercontent.com/130s/ARIAC/feature-cleanup-dependency/dev.repos -O $workspace_in_container/src/dev.repos \
    && cd $workspace_in_container && vcs import --debug --skip-existing $workspace_in_container < $workspace_in_container/src/dev.repos
# Workaround for apt cannot be run https://stackoverflow.com/a/57930100/577001
USER root
RUN echo "DEBUG: ${ROSDISTRO}" \
    && source /opt/ros/${ROSDISTRO}/setup.bash \
    && cd $workspace_in_container \
    && sudo apt update && rosdep update && rosdep install -y --from-paths src --ignore-src \
    && catkin_make
USER $USERNAME

# setup entrypoint
COPY $ENTRY_POINT /
#RUN chmod +x /$ENTRY_POINT

ENTRYPOINT ["/$ENTRY_POINT"]
