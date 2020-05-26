/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PIDDriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  private PIDDriveBase drive;
  private int count = 0;

  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(PIDDriveBase drive, double angle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.ANGLE_P, 0, Constants.ANGLE_D),
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
    getController().setTolerance(Constants.ANGLE_TOLERANCE);
  }

  @Override
  public void initialize() {
    drive.resetAngle();
    m_controller.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (getController().atSetpoint()) {
      count ++;
    }
    else {
      count = 0;
    }
    return count >= 10;
  }
}
