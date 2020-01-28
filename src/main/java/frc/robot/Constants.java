/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // OI Constants

    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
   
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;

    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;

    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;

    public static final int A_BUTTON = 0;
    public static final int B_BUTTON = 1;
    public static final int X_BUTTON = 2;
    public static final int Y_BUTTON = 3;

    public static final int LEFT_BUMPER = 4;
    public static final int RIGHT_BUMPER = 5;

    public static final int BACK_BUTTON = 6;
    public static final int START_BUTTON = 7;

    public static final double MOTOR_DEADZONE = 0.1675;

    // Climber Motor Constants
    public static final double CLIMBER_POWER = 0.5;
    public static final int CLIMBER_MOTOR1 = 1;
    public static final int CLIMBER_MOTOR2 = 2;
    public static final int CLIMBER_SOLENOID = 0;

    // Drive Motor Constants
    public static final int RIGHT_MIDDLE = 3;
    public static final int LEFT_MIDDLE = 5;
    public static final int RIGHT_BACK = 2;
    public static final int LEFT_BACK = 6;
    public static final int RIGHT_FRONT = 1;
    public static final int LEFT_FRONT = 7;

    // Arm Motor Constants
    public static final int ARM_MOTOR_LEFT = 6;
    public static final int ARM_MOTOR_RIGHT = 1;
    public static final int ARM_SOLENOID = 2;
     // For ARM_MOTOR_RIGHT the 1 is just a placehold until we can find the motor ID. 

    //Drive To Distance Constants
    public static final double DRIVE_P = 0.001;

    //Claw Motor Constants
    public static final int INTAKE_POWER = 1;
    public static final int OUTPUT_POWER = -1;
    public static final int CLAW_TOP_MOTOR = 2;
    public static final int CLAW_BOTTOM_MOTOR = 2;

    // Color Wheel Constants
    public static final double COLOR_WHEEL_SPIN_SPEED = 0.5;
    public static final double REVERSE_COLOR_WHEEL_SPIN_SPEED = -0.5;
    public static final int ROTATION_COUNTS_NEEDED = 26;
    
    public static final int WHEEL_MOTOR = 1;
}   // For ARM_MOTOR_RIGHT the 1 is just a placehold until we can find the motor ID. 
