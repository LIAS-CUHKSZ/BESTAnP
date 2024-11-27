roscore
rosrun BESTAnP simulator.py
{catkin_workspace}/BESTAnP/yaml/sim_env.yaml

rosrun BESTAnP traj_generator_eight.py 
rosrun BESTAnP traj_generator_circle.py 
rosrun BESTAnP traj_generator_square.py 

rosrun BESTAnP traj_generator_simple.py 
rosrun BESTAnP traj_generator_circle_simple.py 
rosrun BESTAnP traj_generator_square_simple.py 

roscd BESTAnP && rviz -d ./rviz/sim.rviz 

rqt_image_view
# see /sim/sonar/image

rosrun BESTAnP sonar_logger.py 
