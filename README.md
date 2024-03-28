# Moveit Implementation of the [Gplines](https://github.com/rafaelrojasmiliani/gsplines_cpp)

- Generate minimum acceleration, jerk, snap motions on the top of the ompl planner.
- Allows to implement [opstop](https://github.com/rafaelrojasmiliani/opstop_cpp) and have minimum time path-consisten emergancy stop of the robot.


In specific this repo has:
- An adapter plugin to be used with ompl planning pipeline: given the waypoints returned by a planner this computes a minimum-(jerk, snap, whatever) trajectory.
    - **REMARK** minimum-jerk, snap trajectories may not be collision free (they generate overshoots to preserve smoothness). Use the Rojas adapter with a value of $k> 1.0$ to have collision free trajectories.
- A controller which can interact with the FollowJointTrajectoryGspline action.
    - This controller is useful to implement the opstop emergency stopping trajectory

# Requirements

First install the dependencies
```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew"   \
    python3-matplotlib libgtest-dev cmake libeigen3-dev coinor-libipopt-dev wget \
    ros-noetic-hpp-fcl robotpkg-pinocchio
```

Don't forget to export the `robotpkg` paths
```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

You would need  [gsplines](https://github.com/rafaelrojasmiliani/gsplines_cpp) and [opstop](https://github.com/rafaelrojasmiliani/opstop_cpp)

```bash
wget https://github.com/rafaelrojasmiliani/gsplines_cpp/releases/download/master/gsplines-0.0.1-amd64.deb
wget https://github.com/rafaelrojasmiliani/opstop_cpp/releases/download/master/opstop-0.0.1-amd64.deb
sudo dpkg -i gsplines-0.0.1-amd64.deb  opstop-0.0.1-amd64.deb
```


If you are working with python and developing with gcc-11 you will need the binaries compiled with the same version of gcc.

```bash
wget https://github.com/rafaelrojasmiliani/gsplines_cpp/releases/download/master/gsplines-0.0.1-gcc-11-amd64.deb
wget https://github.com/rafaelrojasmiliani/opstop_cpp/releases/download/master/opstop-0.0.1-gcc-11-amd64.deb
sudo dpkg -i gsplines-0.0.1-gcc-11-amd64.deb  opstop-0.0.1-gcc-11-amd64.deb

```
