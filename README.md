worlds

http://data.nvision2.eecs.yorku.ca/3DGEMS/

https://github.com/srl-freiburg/pedsim_ros
http://pedsim.silmaril.org/

<!-- <plugin name="auto_sliding_door_1" filename="libauto_sliding_door.so">
  <door_name>door1</door_name>
</plugin> -->

rosservice call /gazebo/apply_body_wrench '{body_name: "door3::door_link", wrench: { force: { x: 0, y: 1, z: 0.0 } }, duration: -1 }'
