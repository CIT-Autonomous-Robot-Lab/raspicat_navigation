#!/bin/bash -e

echo -n \
'<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="model"         default="$(find raspicat_description)/urdf/raspicat.urdf.xacro"/>
  <arg name="x_init_pos"    default="0.0"/>
  <arg name="y_init_pos"    default="0.0"/>
  <arg name="z_init_pos"    default="0.0"/>
  <arg name="yaw_init_pos"  default="0.0"/>
  <arg name="open_gui"      default="true"/>
  
  <!-- Prameters -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)"/>
  
  <!-- Nodes -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name"    value="$(find raspicat_map2gazebo)config/world/Free_Name.world"/>
    <arg name="paused"        value="false"/>
    <arg name="use_sim_time"  value="true"/>
    <arg name="gui"           value="$(arg open_gui)"/>
    <arg name="headless"      value="false"/>
    <arg name="debug"         value="false"/>
  </include>

  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" 
    args="-urdf -model raspicat -x $(arg x_init_pos) -y $(arg y_init_pos) -z $(arg z_init_pos) -Y $(arg yaw_init_pos) -param robot_description"/>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <include file="$(find raspicat_gazebo)/launch/raspicat_simulation.launch"/>
</launch>' | sed -e 's/Free_Name/'$1'/g' > $(rospack find raspicat_map2gazebo)/launch/raspicat_$1_world.launch

sleep 'infinity'