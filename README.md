worlds

# referências
http://data.nvision2.eecs.yorku.ca/3DGEMS/
https://github.com/srl-freiburg/pedsim_ros
http://pedsim.silmaril.org/

<!-- <plugin name="auto_sliding_door_1" filename="libauto_sliding_door.so">
  <door_name>door1</door_name>
</plugin> -->





# Start recepcionist task
rosservice call /task_control/recepcionist/start "{}"

# usado para publicar uma nova posição do ator no gazebo
rostopic pub --once /actor_control/<actor_name>/pose geometry_msgs/Pose ...

# usado para abrir uma porta
rostopic pub --once /sliding_door_control/<door_name>/command std_msgs/String "data: 'open'"

# usado para fechar uma porta
rostopic pub --once /sliding_door_control/<door_name>/command std_msgs/String "data: 'close'"




## TODO:
Passar subscribers para services ou actions
sliding_door com exclusão de colisão no cenário e centralizado no vão da porta
ActorControl.msg -> passa std_msgs/String para string
melhorar performance temporal
trocar skin do actor dinamicamente
passar forniture de fei_k5_forniture para models
adicionar regiões
