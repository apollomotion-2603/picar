export ROS_DOMAIN_ID=42
source install/setup.bash
LD_LIBRARY_PATH=/home/apollomotion/acados/lib:$LD_LIBRARY_PATH timeout 3 ros2 run mpc_pkg mpc_node --ros-args --params-file src/mpc_pkg/config/nmpc.yaml
