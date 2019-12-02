# motion-estimation
This repo contains uses three different methods to estimate the motion of a robot.

## Files
`wheel_odometry.m` estimates the motion of a robot using simple wheel odometry. It also compares the map built using ground truth data and estimated motion. <br/> 
`occupancy_grid.m` builds an occupancy grid which will be used in `particle_filter.m` to estimate a robot's position.<br/>
`particle_filter.m` estimates the motion of a robot using a particle filter approach.<br/>
`stereo_vision.m` estimates the motion of a robot using stereo visual odometry.<br/>

