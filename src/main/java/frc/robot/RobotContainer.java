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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.CheesyDrive;
import frc.robot.commands.ExtendClimberSequence;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.Output;
import frc.robot.commands.PullUpRobot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Vision;
import frc.robot.utils.AutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  private final Joystick operatorController = new Joystick(Constants.OPERATOR_CONTROLLER);
  private final JoystickButton operatorControllerLeftBumper = new JoystickButton(operatorController,
      Constants.LEFT_BUMPER);
  private final JoystickButton operatorControllerRightBumper = new JoystickButton(operatorController,
      Constants.RIGHT_BUMPER);
  private final JoystickButton driverControllerRightBumper = new JoystickButton(driverController,
      Constants.RIGHT_BUMPER);
  private final JoystickButton operatorControllerYButton = new JoystickButton(operatorController,
      Constants.Y_BUTTON);
  private final JoystickButton operatorControllerBButton = new JoystickButton(operatorController,
      Constants.B_BUTTON);
  private final JoystickButton operatorControllerXButton = new JoystickButton(operatorController,
      Constants.X_BUTTON);
  private final JoystickButton operatorControllerAButton = new JoystickButton(operatorController,
      Constants.A_BUTTON);
  private final POVButton operatorControllerDPadUp = new POVButton(operatorController, 0);
  private final POVButton operatorControllerDPadLeft = new POVButton(operatorController, 270);
  private final POVButton operatorControllerDPadDown = new POVButton(operatorController, 180);
  private final POVButton operatorControllerDPadRight = new POVButton(operatorController, 90);
  

  // Disable the 2019 drive when testing ColorWheel, suggested by
  // private ColorWheel colorWheel = new ColorWheel();
  private Climber climber = new Climber();
  private Arm arm = new Arm();
  private DriveBase drive = new DriveBase();
  // private Drive2019 drive = new Drive2019();
  private Vision vision = new Vision();
  private Claw claw = new Claw();

  private AutoChooser autoChooser = new AutoChooser(drive, arm, claw);

  private double correctDeadzone(double value) {
    double correctedValue = 0;
    if (Math.abs(value) > Constants.MOTOR_DEADZONE) {
      if (value < 0) {
        correctedValue = ((value + Constants.MOTOR_DEADZONE) / (1 - Constants.MOTOR_DEADZONE));
      }
      if (value > 0) {
        correctedValue = ((value - Constants.MOTOR_DEADZONE) / (1 - Constants.MOTOR_DEADZONE));
      }
    }
    return correctedValue;
  }

  private double getDriverLeftYAxis() {
    return -correctDeadzone(driverController.getRawAxis(Constants.LEFT_Y_AXIS));
  }

  private double getDriverLeftXAxis() {
    return correctDeadzone(driverController.getRawAxis(Constants.LEFT_X_AXIS));
  }

  private double getDriverRightYAxis() {
    return -correctDeadzone(driverController.getRawAxis(Constants.RIGHT_Y_AXIS));
  }

  private double getDriverRightXAxis() {
    return correctDeadzone(driverController.getRawAxis(Constants.RIGHT_X_AXIS));
  }

  private double getOperatorLeftYAxis() {
    return -correctDeadzone(operatorController.getRawAxis(Constants.LEFT_Y_AXIS));
  }

  private double getOperatorLeftXAxis() {
    return correctDeadzone(operatorController.getRawAxis(Constants.LEFT_X_AXIS));
  }

  private double getOperatorRightYAxis() {
    return -correctDeadzone(operatorController.getRawAxis(Constants.RIGHT_Y_AXIS));
  }

  private double getOperatorRightXAxis() {
    return correctDeadzone(operatorController.getRawAxis(Constants.RIGHT_X_AXIS));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    drive.setDefaultCommand(new CheesyDrive(drive, () -> getDriverLeftYAxis(), () -> getDriverRightXAxis(), Constants.HIGH_POWER_DRIVE));
    driverControllerRightBumper.whileHeld(new CheesyDrive(drive, () -> getDriverLeftYAxis(), () -> getDriverRightXAxis(), Constants.LOW_POWER_DRIVE));

    // operatorControllerRightBumper.whenPressed(new PositionControl(colorWheel));
    // operatorControllerLeftBumper.whenPressed(new RotationControl(colorWheel,
    // Constants.ROTATION_COUNTS_NEEDED, operatorController));
    // drive.setDefaultCommand(new CheesyDrive(drive, () -> getDriverLeftYAxis(), ()
    // -> getDriverRightXAxis()));
    // operatorControllerLeftBumper.toggleWhenPressed(new
    // StopCompressor(pneumatics));

    //operatorControllerLeftBumper.and(operatorControllerRightBumper).and(operatorControllerYButton)
    //    .whenActive(new ExtendClimberSequence(climber));

    operatorControllerLeftBumper.and(operatorControllerRightBumper).and(operatorControllerYButton)
     .whenActive(new ExtendClimberSequence(climber));
    operatorControllerXButton.whenHeld(new PullUpRobot(climber));
    //operatorControllerLeftBumper.whenPressed(new ExtendClimberSequence(climber).withTimeout(Constants.CLIMBER_RELEASE_DELAY));
    //operatorControllerRightBumper.whenPressed(new EngageClimber(climber));
    operatorControllerDPadRight.whenPressed(new MoveArmToPosition(arm, Constants.ARM_SCORE_POSITION, false));
    operatorControllerDPadUp.whenPressed(new MoveArmToPosition(arm, Constants.ARM_LOAD_POSITION, false));
    operatorControllerDPadDown.whenPressed(new MoveArmToPosition(arm, Constants.ARM_HOME_POSITION, true));
    //operatorControllerDPadLeft.whenPressed(new MoveArm(arm, () -> getOperatorRightYAxis()));
    operatorControllerAButton.whenHeld(new Intake(claw));
    operatorControllerBButton.whenHeld(new Output(claw));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.GenerateAuto();
    
  }
}