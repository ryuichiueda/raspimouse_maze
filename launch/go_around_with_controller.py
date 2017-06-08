<launch>
  <include file="$(find raspimouse_ros_2)/launch/raspimouse.launch" /> 
  <node pkg="raspimouse_maze" name="go_around" type="go_around_with_controller.py" required="true" output="screen" />
</launch>
