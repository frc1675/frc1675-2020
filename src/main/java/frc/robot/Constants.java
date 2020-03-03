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
    
    // Autonomous Constants
    public static final double TICKS_PER_INCH = 13.6; //not accurate
    public static final double TOLERANCE = 1;

    // OI Constants
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;
   
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;

    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;

    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;

    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;

    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;

    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // Joystick buttons (clicking them in)
    public static final int LEFT_JOYSTICK_BUTTON = 9;
    public static final int RIGHT_JOYSTICK_BUTTON = 10;

    public static final double MOTOR_DEADZONE = 0.1675;

    // Climber Motor Constants
    public static final double CLIMBER_POWER = 0.4;
    public static final int CLIMBER_MOTOR_RIGHT = 6;
    public static final int CLIMBER_MOTOR_LEFT = 5;
    public static final int CLIMBER_RETRACT_SOLENOID = 1; //just got switched
    public static final int CLIMBER_RELEASE_SOLENOID = 0;
    public static final double CLIMBER_RELEASE_DELAY = 0.25;

    public static final double CLIMBER_EXTEND_TIME = 2;

    // Drive Motor Constants
    public static final int RIGHT_MIDDLE = 3;
    public static final int LEFT_MIDDLE = 5;
    public static final int RIGHT_BACK = 3;
    public static final int LEFT_BACK = 2;
    public static final int RIGHT_FRONT = 4;
    public static final int LEFT_FRONT = 1;

    // Arm Motor Constants
    public static final int ARM_MOTOR_LEFT = 7;
    public static final int ARM_MOTOR_RIGHT = 8;
    public static final int ARM_SOLENOID = 2;
    public static final double ARM_POWER = 0.1;
    public static final double ARM_TOLERANCE = 5;
    public static final double ARM_P = .006;
    public static final double ARM_SCORE_POSITION = 115;
    public static final double ARM_LOAD_POSITION = 130;
    public static final double ARM_HOME_POSITION = 42; 
    //Drive To Distance Constants
    public static final double DRIVE_P = 0.001;
    public static final double LOW_POWER_DRIVE = 0.5;
    public static final double HIGH_POWER_DRIVE = 1.0;

    //Claw Motor Constants
    public static final double INTAKE_POWER = -1;
    public static final double OUTPUT_POWER = 1;
    public static final int CLAW_MOTOR = 9;

    // Color Wheel Constants
    public static final double COLOR_WHEEL_SPIN_SPEED = 0.5;
    public static final double REVERSE_COLOR_WHEEL_SPIN_SPEED = -0.5;
    public static final int ROTATION_COUNTS_NEEDED = 26;
    public static final int THROTTLE_TIMER = 5;
    public static final int WHEEL_MOTOR = 1;
}   // For ARM_MOTOR_RIGHT the 1 is just a placehold until we can find the motor ID. 
