/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drive2019;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  Drive2019 drive;

  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(Drive2019 drive, double angle) {
    super(
        // The controller that the command will use
        new PIDController(0.00278, 0, 0),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          drive.setRightMotors(-output);
          drive.setLeftMotors(output);
          // Use the output here
        });
    this.drive = drive;
    addRequirements(drive);
    getController().enableContinuousInput(-180, 180);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10);
  }

  @Override
  public void initialize() {
    drive.resetAngle();
    m_controller.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
