/**
 * Author: Dagmar Prokopova
 * File: README.txt
 * Description: Instructions on running the Ball Picker application
 * Bachelor's thesis, 2013/2014
 *
 */

 *** Running in simulator with robot model TurtleBUT2:
     1) start the simulation with robot model inside
        roslaunch tb_gazebo robot_empty_world.launch
     2) start the gmapping node and move_base
        roslaunch ball_picker tb_gmap
     3) start the Ball Picker application
        roslaunch ball_picker ball_picker_simulator.launch 

 *** Running with real TurtleBUT2:
     1) bring up the robot
        roslaunch tb_bringup robot.launch
     2) start the gmapping node and move_base
        roslaunch tb_navigation slam.launch
     3) start the Ball Picker application
        roslaunch ball_picker ball_picker.launch
