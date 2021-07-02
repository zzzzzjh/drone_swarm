##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch nlink_parser linktrack.launch"' \
--tab -e 'bash -c "sleep 2; roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch drone_command px4_pos_estimator_uwb.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch drone_command px4_sender.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun drone_command square; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun drone_command set_mode; exec bash"' \

