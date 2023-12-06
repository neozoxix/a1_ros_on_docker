FROM ros:noetic-perception-focal
# fixed a key issue using the method mentioned in https://github.com/PRBonn/bonnetal/commit/0ab883b8356954c3e57a07a73c30bbac8f035a05
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  apt update 

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive 

# install necessary dependencies 
RUN apt-get update \
  && apt-get install -y \
      apt-utils \
      autoconf \
      automake \
      build-essential \
      dbus-x11 \
      dos2unix \
      clang \
      cmake \
      curl \
      g++ \
      gcc \
      gdb \
      gdb valgrind \
      git \
      htop \
      ipmiutil \
      liblcm-dev \
      locales-all \
      make \
      mesa-utils \
      net-tools \
      ninja-build \
      rsync \
      software-properties-common \
      subversion \
      ssh \
      tar \
      xterm \
      vim \
      wget 

RUN apt-get update \
  && apt-get install -y \
      python3-catkin-tools \
      python3-osrf-pycommon \
      python3-matplotlib \
      python3-rospkg 

RUN apt-get update \
  && apt-get install -y \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libopenmpi-dev \
      libssl-dev \
      coinor-libipopt-dev \
      && apt-get clean

# For intel iris driver
RUN add-apt-repository ppa:kisak/kisak-mesa -y && apt update && apt upgrade -y

  
# # install cmake 3.20 (Akhdan : Use newer one)
# WORKDIR /tmp 
# RUN apt-get -y install wget libtool
# RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
# RUN tar -zxvf cmake-3.20.2.tar.gz
# RUN ls
# WORKDIR /tmp/cmake-3.20.2
# RUN ./bootstrap
# RUN make -j8
# RUN make install

# Install cmake 3.26.4
RUN git clone https://github.com/Kitware/CMake.git && \
cd CMake && git checkout tags/v3.26.4 && ./bootstrap --parallel=8 && make -j8 && make install && \
cd .. && rm -rf CMake

# Install new paramiko (solves ssh issues) (Akhdan : Dont use it for now)
# RUN apt-add-repository universe
# RUN apt-get update && apt-get install -y python3-pip python build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*
# RUN /usr/bin/yes | pip3 install --upgrade pip
# RUN /usr/bin/yes | pip3 install --upgrade virtualenv
# RUN /usr/bin/yes | pip3 install --upgrade paramiko
# RUN /usr/bin/yes | pip3 install --ignore-installed --upgrade numpy protobuf
# RUN /usr/bin/yes | pip3 install --upgrade setuptools

# Locale (Akhdan : Dont use it for now)
# RUN locale-gen en_US.UTF-8  
# ENV LANG en_US.UTF-8  
# ENV LANGUAGE en_US:en  
# ENV LC_ALL en_US.UTF-8

# install python 3.8 
RUN apt-get update && \
    apt-get install -y python3.8
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
RUN curl https://bootstrap.pypa.io/pip/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN rm get-pip.py
RUN pip --version
# RUN pip install pybullet
# RUN pip install tensorflow
# RUN pip install numpy
# RUN pip install scipy

# add OSQP (Akhdan : Dont use it for now)
# follow https://osqp.org/docs/get_started/sources.html#build-from-sources to install OSQP from sources
# WORKDIR /tmp
# RUN git clone --recursive https://github.com/oxfordcontrol/osqp
# WORKDIR /tmp/osqp
# RUN mkdir build 
# WORKDIR /tmp/osqp/build
# RUN cmake -G "Unix Makefiles" ..
# RUN cmake --build .
# RUN cmake --build . --target install

# add OSQP-python (Akhdan : Dont use it for now)
# RUN pip install osqp
# RUN apt update
# RUN apt install -y python3-pip

# add osqp-eigen (Akhdan : Dont use it for now)
# WORKDIR /tmp
# RUN git clone https://github.com/robotology/osqp-eigen.git
# WORKDIR /tmp/osqp-eigen
# RUN mkdir build 
# WORKDIR /tmp/osqp-eigen/build
# RUN cmake ../
# RUN make
# RUN make install

# lcm dependencies, notice ContactImplicitMPC.jl needs 2.68.3, we try 2.64.6 first
RUN apt-cache madison libglib2.0-dev
RUN apt-get install -y libglib2.0-0 libglib2.0-dev
# WORKDIR /tmp
# RUN wget http://archive.ubuntu.com/ubuntu/pool/main/g/glib2.0/libglib2.0-0_2.68.4-1ubuntu1_amd64.deb
# RUN dpkg -i libglib2.0-0_2.68.4-1ubuntu1_amd64.deb

# make unitree workspace
ENV SUPPORT_WS=/root/support_files
ENV UNITREE_WS=/root/unitree_ws
RUN mkdir -p $SUPPORT_WS 
RUN mkdir -p $UNITREE_WS/src
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init;"
# install unitree sdk dependencies
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd ${SUPPORT_WS}/lcm && \
    git checkout tags/v1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 && \
    make install 

# notice we must use v3.2 
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd ${SUPPORT_WS}/unitree_legged_sdk && git checkout v3.2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 

# clone yue env
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/yuefufufu/a1_setup_ubuntu20.git
RUN cd a1_setup_ubuntu20

RUN apt install -y ros-noetic-desktop-full

RUN apt-get install -y \
    ros-noetic-controller-interface \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-move-base-msgs

RUN apt-get install -y ros-noetic-teleop-twist-keyboard

# install necessary dependencies 
# RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
#       else export USE_PROC=$(($(nproc)/2)) ; fi && \
#       apt-get update && apt-get install -y \
#       ros-${ROS_DISTRO}-ros-control \
#       ros-${ROS_DISTRO}-gazebo-ros \
#       ros-${ROS_DISTRO}-joy \
#       ros-${ROS_DISTRO}-ros-controllers \
#       ros-${ROS_DISTRO}-robot-state-publisher \
#       ros-${ROS_DISTRO}-xacro \
#       ros-${ROS_DISTRO}-move-base-msgs \
#       ros-${ROS_DISTRO}-turtlesim \
#       ros-${ROS_DISTRO}-rqt \
#       ros-${ROS_DISTRO}-rqt-common-plugins \
#       ros-${ROS_DISTRO}-gazebo-ros-control

# WORKDIR $UNITREE_WS/src
# RUN git clone https://github.com/ShuoYangRobotics/unitree_ros.git
WORKDIR $UNITREE_WS
# there are some non ascii code in this file that prevents docker from catkin build the file
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
ENV UNITREE_SDK_VERSION=3_2
ENV UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk
ENV ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk
ENV UNITREE_PLATFORM=amd64
RUN echo "#unitree config" >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export ROS_PACKAGE_PATH=${UNITREE_WS}:${ROS_PACKAGE_PATH}' >> ~/.bashrc
RUN echo 'export GAZEBO_PLUGIN_PATH=${UNITREE_WS}/devel/lib:${GAZEBO_PLUGIN_PATH}' >> ~/.bashrc
RUN echo 'export LD_LIBRARY_PATH=${UNITREE_WS}/devel/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
RUN echo '# 3_1, 3_2' >> ~/.bashrc
RUN echo "export export UNITREE_SDK_VERSION=3_2" >> ~/.bashrc
RUN echo "export UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk" >> ~/.bashrc
RUN echo "export ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk" >> ~/.bashrc
RUN echo '# amd64, arm32, arm64' >> ~/.bashrc
RUN echo "export UNITREE_PLATFORM=\"amd64\"" >> ~/.bashrc
RUN echo "source ${UNITREE_WS}/devel/setup.bash" >> ~/.bashrc

# RUN echo "export ROS_MASTER_URI=http://192.168.123.2:11311;export ROS_IP=192.168.123.2;export ROS_HOSTNAME=192.168.123.2" >> ~/.bashrc
# compile just unitree ros unitree_legged_msgs
# RUN ls $UNITREE_WS/src/unitree_ros
RUN cp -fr ~/support_files/a1_setup_ubuntu20/unitree_ros ~/unitree_ws/src/
RUN cp -fr ~/support_files/a1_setup_ubuntu20/unitree_ros_to_real ~/unitree_ws/src/
RUN cp -fr ~/support_files/a1_setup_ubuntu20/unitree_guide ~/unitree_ws/src/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build unitree_legged_msgs;"

# To use rosparam load yaml files
RUN pip install pyyaml