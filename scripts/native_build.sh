rosdep update
rosdep install -i --from-path /home/ubuntu/2023WaterCode/fishROS_ws/src --rosdistro humble -y
cd /home/ubuntu/2023WaterCode/fishROS_ws/ 
colcon build --parallel-workers 4 