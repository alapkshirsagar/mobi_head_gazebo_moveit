# mObi head
# Dockerfile for Ubuntu 14.04 + ROS Indigo for mObi head software development

# Credit goes to Cetin Mericli

FROM osrf/ros:indigo-desktop-full
MAINTAINER Tekin Mericli tekin.mericli@gmail.com

# nvidia-docker uses this label to find which docker volume to mount 
# for exposing the host NVidia drivers
# not necessary if inheriting from nvidia/cuda images

LABEL com.nvidia.volumes.needed="nvidia_driver"

#RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
#    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}


# setup environment

RUN apt-get update && \
    apt-get install -y --fix-missing --no-install-recommends \
    sudo \
    net-tools \
    locales \
    module-init-tools \
    binutils \
    wget \
    vim \
    screen \
    nano \
    git \
    mercurial \
    bash-completion \
    ipython \
    aptitude \
    htop \
    mesa-utils \
    mlocate \
    gdb \
    locate \
    htop \
    linux-tools-common \
    usbutils \
    xterm \
    iperf \
    google-perftools \
    lsb-release \
	iputils-ping \
    && apt-get dist-upgrade -y

RUN updatedb

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive


# re-evaluate if this is principled enough ADD ./mobihead-build /usr/bin/mobihead-build
ADD ./mobihead-build /usr/bin/mobihead-build



# mObi head dependencies

RUN apt-get install --no-install-recommends -y --force-yes \
	libpcap-dev \
	python-argcomplete \
	python-catkin-tools \
	ros-indigo-moveit \
	ros-indigo-rosserial \
	ros-indigo-rosserial-python \
	ros-indigo-opencv-candidate \
	ros-indigo-opencv3 \
	ros-indigo-tf2-ros \
	ros-indigo-tf2-web-republisher \
	ros-indigo-ros-control \
	ros-indigo-effort-controllers \
	ros-indigo-joint-state-controller \
	ros-indigo-joint-trajectory-controller \
	ros-indigo-dynamixel-motor \
	ros-indigo-rosbridge-suite \
	ros-indigo-web-video-server \
	qv4l2 \
	usbutils \
	v4l-utils \
	libv4l-dev \
	libpopt-dev \
	libmrpt-dev



# mObi head user
ARG MOBIHEAD_USER=mobihead
ARG CUSTOM_UID=1000
ENV CUSTOM_UID ${CUSTOM_UID}
ENV MOBIHEAD_USER ${MOBIHEAD_USER}
RUN adduser -u $CUSTOM_UID --disabled-password --gecos '' $MOBIHEAD_USER
RUN adduser $MOBIHEAD_USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
ENV HOME /home/$MOBIHEAD_USER
WORKDIR ${HOME}


# for potential video camera access
RUN usermod -a -G dialout $MOBIHEAD_USER
RUN usermod -a -G video $MOBIHEAD_USER

USER $MOBIHEAD_USER

RUN rosdep update

ADD vimrc ${HOME}/.vimrc
ADD screenrc ${HOME}/.screenrc

RUN sed -i -- 's/#force_color_prompt=yes/force_color_prompt=yes/g' ~/.bashrc
#RUN /bin/bash -c "echo 'exec >/dev/tty 2>/dev/tty </dev/tty' >> ~/.bashrc"

ADD ./README $HOME/README
ADD ./bash_additions /tmp/bash_additions
RUN /bin/bash -c "echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export QT_GRAPHICSSYSTEM=raster' >> ~/.bashrc"

# Set up ROS paths and a catkin workspace for the robot code to run in

RUN . /opt/ros/indigo/setup.sh \
&& cat /tmp/bash_additions >> ~/.bashrc \
&& echo "export ROS_PACKAGE_PATH=~/code/catkin_ws:$ROS_PACKAGE_PATH" >> ~/.bashrc  \
&& echo "export ROS_WORKSPACE=~/code/catkin_ws" >> ~/.bashrc   \
&& . ~/.bashrc \
&& mkdir -p ~/code/catkin_ws/src/mobi_head_control \
&& mkdir -p ~/code/catkin_ws/src/mobi_head_moveit \
&& cd ~/code/catkin_ws/ \
&& catkin_make \
&& echo "source ~/code/catkin_ws/devel/setup.bash"
