##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4 posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch drone_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch drone_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun drone_command square; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun drone_command set_mode; exec bash"' \

