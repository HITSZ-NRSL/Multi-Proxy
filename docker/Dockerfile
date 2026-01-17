FROM osrf/ros:noetic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive


RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
RUN sed -i '/security.ubuntu.com/d' /etc/apt/sources.list
RUN echo "deb https://mirrors.ustc.edu.cn/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN apt update && apt-get install -y --no-install-recommends unzip curl wget g++ build-essential vim git bash-completion

RUN curl -L -o gtsam-4.2.0.tar.gz https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.tar.gz
RUN tar -xzf gtsam-4.2.0.tar.gz
RUN rm gtsam-4.2.0.tar.gz && cd gtsam-4.2.0 && mkdir build && cd build && \
    cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_TBB=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && ldconfig

RUN apt update && apt install -y iproute2 iputils-ping

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo "alias cl='clear && source ~/.bashrc'" >> /root/.bashrc
RUN echo "alias ckm='catkin_make'" >> /root/.bashrc
RUN echo "alias ckb='catkin build'" >> /root/.bashrc
RUN echo "alias srcd='source devel/setup.bash'" >> /root/.bashrc

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
