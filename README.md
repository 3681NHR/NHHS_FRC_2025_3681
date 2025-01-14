# 2025 robot
 - PR all fetures to dev unless you have explicit aproval from an admin and the code has been tested on the robot
 - stable branch will always be fully functional on-robot
 - dev branch will often be functional on-robot
 - PRs from dev to stable require admin review and **must pass all tests on the robot**
## what does what

- Main.java - dont touch
- Robot.java - rarely used
- RobotContainer.java - robot init and trigger bindings
handles bindings and input
- Constants.java - constants and cool numbers
- subsystems
    - Vision.java - handles PhotonVision
    - SwerveDriveSubsystem.java - handles swerve drive
 
 - utils
     - common across most repos
     - generic functions     

## CAN ID conventions
most CAN devices should have a first digit that defines what the device is used for, followed by an identifier that determins what device it is. 
0 should not be used, as it is reserved for unconfigured devices and default values in code
0 - default/placeholder(dont use on robot)
1 - swerve drive motor
2 - swerve steer motor
3 - util
4 - arm extention
5 - arm pivot
6 - misc. motor

## todo
 - auto aim buttons
 - prox commands
 - rot snap buttons

## on-robot tests
 - ### drive
    - all swerve modules point in correct directions and follow optimal path on blocks and ground when you:
        - translate forward, back, left, right
        - move left stick in a full circle slowly
        - rotate left
        - rotate right
        - lock pose
      
    - robot must move in intended way when you:
         - translate in all directions
         - rotate left and right
         - set angle with direct angle mode
         - spin while translating in FOD mode
     
- confirm reset gyro works
- confirm gyro angle changes properly as robot rotates
- confirm odometry follows robot motions
- confirm uniform/correct wheel speeds
     
 - confirm vision
    - dashboard vision output is connected and updates
    - photon vision web dashboard shows correct outputs
    - apriltag detection works and reads relitive position/angle correctly
     
 - confirm all rumble ques are called and have correct output
 - confirm correct dashboard layout
     
- ### dashboard
    - confirm correct timer values
        - up time
        - dynamic remaining
        - auto and teleop remaining
        - auto and teleop up time
        - auto and teleop total
        - auto and teleop start times
        - correct coloring
        
    - confirm widgets:
        - camera feeds
        - odometry field
        - fod/direct angle indicators
        - pdp
        - swerve widget
   