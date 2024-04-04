ARG BASEIMAGE
FROM ${BASEIMAGE}
ARG ROS_DISTRO


RUN --mount=type=bind,source=./,target=/workspace/src,rw \
    cd /workspace \
    && wget https://github.com/rafaelrojasmiliani/gsplines_cpp/releases/download/master/gsplines-0.0.1-gcc-11-amd64.deb \
    && wget https://github.com/rafaelrojasmiliani/opstop_cpp/releases/download/master/opstop-0.0.1-gcc-11-amd64.deb \
    && dpkg -i gsplines-0.0.1-gcc-11-amd64.deb  opstop-0.0.1-gcc-11-amd64.deb \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin config --install \
                -DCMAKE_BUILD_TYPE=Release \
                -DCATKIN_SKIP_TESTING=ON \
                --install-space /opt/ros/${ROS_DISTRO} \
    &&  catkin build
