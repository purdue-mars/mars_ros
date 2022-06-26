search_dir=/home/ruppulur/catkin_ws/src/icra_2022_mars/mars_sim/nist_board_gazebo/models/mesh
for fullfile in "$search_dir"/*
do
  in=${fullfile}
  out="${fullfile%.*}.obj"
  ctmconv ${in} ${in} --scale 0.001 
done
