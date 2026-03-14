# source /opt/ros/humble/setup.bash
cmake -B build
make -C build -j `nproc`