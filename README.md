# filter_octomap
Helpers for OctoMap used in https://github.com/kingjin94/enhanced_simulation. Main functions are generating a initial OctoMap filled with prior information and generating object proposals out of the OctoMap during exploration. Additionally, it publishes the OctoMap to MoveIt's planning scene and calculates the OctoMap's entropy as a measure of the mapping progress.

## Before Usage
One has to construct the prefilled maps first before using this package

1.) Default with 2 cm resolution is constructed with makeMaps.sh

2.) For others replace the 0.02 with your desired resolution

3.) For adapting the prefilled areas please change src/octomap_generate_empty_start.cpp
