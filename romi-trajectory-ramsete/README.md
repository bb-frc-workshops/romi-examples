# Romi Trajectory Follower

## Description
This example showcases how to use the WPILib `RamseteCommand` to make your Romi follow a predefined trajectory. These trajectories can be hand crafted, or generated using a tool like PathWeaver.

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

## Additional Hardware Required
None

## Additional Configuration Required
- Ensure that the gyro has been [calibrated using the web UI](https://docs.wpilib.org/en/stable/docs/romi-robot/web-ui.html#imu-calibration)
- For best results, you should [run a characterization on your Romi](../romi-characterization) since there might be slight variations between Romis (due to assembly, mechanical difference, etc)

## Additional Code Setup
The trajectory can be modified by editing the `generateRamseteCommand` method in `RobotContainer.java`