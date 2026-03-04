FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    ros-noetic-diagnostic-updater \
    ros-noetic-tf \
    libboost-program-options-dev \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/.ssh && ssh-keyscan -t rsa github.com >> /root/.ssh/known_hosts

WORKDIR /root/catkin_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/net-ft-ros.git

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]