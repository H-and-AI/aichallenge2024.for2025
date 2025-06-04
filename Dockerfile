# FROM osrf/ros:humble-desktop AS common
FROM ghcr.io/automotiveaichallenge/autoware-universe:humble-latest AS common

# FIX/ERROR
# ROS
# Update gpg key
# ROS 2 Documentationに記載されている方法では /etc/apt/sources.list.d/ros2.list
# ros:jazzy-ros-base などのDockerイメージでは /etc/apt/sources.list.d/ros2-latest.list
RUN \
  rm -rf /etc/apt/sources.list.d/ros2-latest.list && \
  rm -rf /etc/apt/sources.list.d/ros2.list && \
  apt-get update -q && \
  apt-get install -yq --no-install-recommends curl && \
  curl -sS https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $( . /etc/os-release && echo $UBUNTU_CODENAME ) main" > /etc/apt/sources.list.d/ros2.list && \
  rm -rf /var/lib/apt/lists/*

RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | tee -a /etc/apt/sources.list > /dev/null && apt-get update
RUN apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
RUN apt-get -y install iproute2
RUN apt-get -y install wmctrl
RUN apt-get -y install ros-humble-rqt-tf-tree
RUN apt-get -y install ros-humble-rqt-graph

RUN apt install zenoh-bridge-ros2dds terminator -y
RUN apt install arp-scan -y

# PATH="$PATH:/root/.local/bin"
# PATH="/usr/local/cuda/bin:$PATH"
ENV XDG_RUNTIME_DIR=/tmp/xdg
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///opt/autoware/cyclonedds.xml

COPY vehicle/cyclonedds.xml /opt/autoware/cyclonedds.xml

FROM common AS dev

RUN echo 'export PS1="\[\e]0;(AIC_DEV) ${debian_chroot:+($debian_chroot)}\u@\h: \w\a\](AIC_DEV) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /etc/skel/.bashrc
RUN echo 'cd /aichallenge' >> /etc/skel/.bashrc
ENV RCUTILS_COLORIZED_OUTPUT=1

FROM common AS eval

ENV RCUTILS_COLORIZED_OUTPUT=0

RUN mkdir /ws
RUN git clone --depth 1 https://github.com/AutomotiveAIChallenge/aichallenge-2024 /ws/repository
RUN mv /ws/repository/aichallenge /aichallenge
RUN rm -rf /aichallenge/simulator
RUN rm -rf /aichallenge/workspace/src/aichallenge_submit
RUN chmod 757 /aichallenge

COPY aichallenge/simulator/ /aichallenge/simulator/
COPY submit/aichallenge_submit.tar.gz /ws
RUN tar zxf /ws/aichallenge_submit.tar.gz -C /aichallenge/workspace/src
RUN rm -rf /ws

RUN bash -c ' \
  source /autoware/install/setup.bash; \
  cd /aichallenge/workspace; \
  rosdep update; \
  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

ENTRYPOINT []
CMD ["bash", "/aichallenge/run_evaluation.bash"]
