worlds

http://data.nvision2.eecs.yorku.ca/3DGEMS/

https://github.com/srl-freiburg/pedsim_ros
http://pedsim.silmaril.org/

<!-- <plugin name="auto_sliding_door_1" filename="libauto_sliding_door.so">
  <door_name>door1</door_name>
</plugin> -->








# usado para publicar uma nova posição do ator no gazebo
rostopic pub --once /actor_control/<actor_name>/pose geometry_msgs/Pose ...

# usado para abrir uma porta
rostopic pub --once /sliding_door_control/<door_name>/command std_msgs/String "data: 'open'"

# usado para fechar uma porta
rostopic pub --once /sliding_door_control/<door_name>/command std_msgs/String "data: 'close'"
