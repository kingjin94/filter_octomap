# octomap_helper
One has to construct the prefilled maps first before using this package

1.) Set desired resolution in src/octomap_generate_empty_start.cpp
2.) catkin_make_isolated in catkin_ws
3.) rosrun filter_ocotmap octomap_generate_empty_start
4.) move resulting .ot file to filter_octomaps/maps/
5.) add resulting map file to the enhanced_sim explorer.launch (where the octomap server node is started)
