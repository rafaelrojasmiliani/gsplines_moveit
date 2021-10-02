# This file tells docker what image must be created
# in order to be ahble to test this library
FROM moveit/moveit:noetic-release

RUN apt clean
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update

# Install packages
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    python3-pip git iputils-ping net-tools netcat screen   less \
                    python3-sympy coinor-libipopt-dev  valgrind \
                     pkg-config exuberant-ctags \
                    liblapack-dev liblapack3 libopenblas-base libopenblas-dev \
                    libgfortran-7-dev cmake libgsl-dev gdb python3-tk libeigen3-dev \
                    libboost-math-dev

COPY vim_installation.bash /
RUN cd / && bash vim_installation.bash
COPY configfiles/vimrc /etc/vim/
COPY configfiles/ycm_extra_conf.py /etc/vim/
#RUN vim -c ':call doge#install()' -c ':q'
RUN chmod 777 /etc/vim/
RUN chmod 777 /etc/vim/vimrc
RUN chmod 777 /etc/vim/bundle

RUN pip3 install setuptools matplotlib Mosek scipy quadpy six cython tk

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    ros-noetic-ifopt  ros-noetic-pinocchio  python3-catkin-tools \
                    ros-noetic-plotjuggler \
                    ros-noetic-joint-trajectory-controller \
                    ros-noetic-joint-trajectory-action \
                    ros-noetic-xacro \
                    ros-noetic-gazebo-ros \
                    ros-noetic-gazebo-ros-control \
                    ros-noetic-joint-state-controller \
                    ros-noetic-position-controllers \
                    ros-noetic-rqt \
                    ros-noetic-rqt-graph \
                    ros-noetic-roslint \
                    ros-noetic-rqt-gui \
                    ros-noetic-rqt-gui-py \
                    ros-noetic-rqt-py-common \
                    ros-noetic-rqt-joint-trajectory-controller \
                    ros-noetic-libfranka \
                    ros-noetic-panda-moveit-config \
                    ros-noetic-combined-robot-hw \
                    ros-noetic-tf-conversions



RUN mkdir -p /aux_ws/src
RUN git clone https://github.com/rafaelrojasmiliani/ur_description_minimal.git /aux_ws/src/ur_description_minimal
RUN git clone https://github.com/tork-a/rqt_joint_trajectory_plot.git /aux_ws/src/rqt_joint_trajectory_plot
RUN git clone --branch noetic-devel https://github.com/rafaelrojasmiliani/franka_ros.git /aux_ws/src/franka_ros
RUN bash -c 'source /opt/ros/noetic/setup.bash && cd /aux_ws && catkin config --install --install-space /opt/ros/noetic/ --extend /opt/ros/noetic/ && catkin build'


# user handling
ARG myuser
ARG myuid
ARG mygroup
ARG mygid
ARG scriptdir
RUN addgroup --gid ${mygid} ${mygroup} --force-badname
RUN adduser --gecos "" --disabled-password  --uid ${myuid} --gid ${mygid} ${myuser} --force-badname
#add user to sudoers
RUN echo "${myuser} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN echo "${myuser}:docker" | chpasswd


RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
WORKDIR /catkinws
RUN chmod 777 /catkinws


RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --install-recommends -o Dpkg::Options::="--force-confnew" \
                    avahi-daemon \
                    avahi-autoipd \
                    openssh-server \
                    isc-dhcp-client \
                    iproute2

RUN service ssh start
RUN echo '[server]' >> /etc/avahi/avahi-daemon.conf
RUN echo 'enable-dbus=no' >> /etc/avahi/avahi-daemon.conf
RUN echo 'domain-name=local' >> /etc/avahi/avahi-daemon.conf
RUN echo 'host-name=gsplines-ros' >> /etc/avahi/avahi-daemon.conf
