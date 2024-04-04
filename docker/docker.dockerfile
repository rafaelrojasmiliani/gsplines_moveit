ARG BASEIMAGE
FROM ${BASEIMAGE}
ARG ROS_DISTRO


RUN --mount=type=bind,source=./,target=/workspace/src,rw \
    cd /workspace \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && export PATH=/opt/openrobots/bin:$PATH \
    && export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    && export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    && export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH \
    && wget https://github.com/rafaelrojasmiliani/gsplines_cpp/releases/download/master/gsplines-0.0.1-gcc-11-amd64.deb \
    && wget https://github.com/rafaelrojasmiliani/opstop_cpp/releases/download/master/opstop-0.0.1-gcc-11-amd64.deb \
    && dpkg -i gsplines-0.0.1-gcc-11-amd64.deb  opstop-0.0.1-gcc-11-amd64.deb \
    && catkin config --install \
                -DCMAKE_BUILD_TYPE=Release \
                -DCATKIN_SKIP_TESTING=ON \
                --install-space /opt/ros/${ROS_DISTRO} \
    &&  catkin build
