/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  
  private double correctDeadZone(double value){
    double correctedValue = 0;
    if(Math.abs(value) < Constants.MOTOR_DEADZONE) {
      if(value > 0){
        correctedValue = (value + Constants.MOTOR_DEADZONE)/(1 - Constants.MOTOR_DEADZONE);
      }
      if(value < 0){
        correctedValue = (value - Constants.MOTOR_DEADZONE)/(1 - Constants.MOTOR_DEADZONE);
      }
    }
    return correctedValue;
  }

  private double getDriverLeftYAxis(){
    return driverController.getRawAxis(Constants.LEFT_Y_AXIS);
  }
  private double getDriverLeftXAxis(){
    return driverController.getRawAxis(Constants.LEFT_X_AXIS);
  }
  private double getDriverRightYAxis(){
    return driverController.getRawAxis(Constants.RIGHT_Y_AXIS);
  }
  private double getDriverRightXAxis(){
    return driverController.getRawAxis(Constants.RIGHT_X_AXIS);
  }
 
  private double getOperatorLeftYAxis(){
    return operatorController.getRawAxis(Constants.LEFT_Y_AXIS);
  }
  private double getOperatorLeftXAxis(){
    return operatorController.getRawAxis(Constants.LEFT_X_AXIS);
  }
  private double getOperatorRightYAxis(){
    return operatorController.getRawAxis(Constants.RIGHT_Y_AXIS);
  }
  private double getOperatorRightXAxis(){
    return operatorController.getRawAxis(Constants.RIGHT_X_AXIS);
  }
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
