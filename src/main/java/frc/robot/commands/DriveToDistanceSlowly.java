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
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveToDistanceSlowly extends PIDCommand {
    private DriveBase drive;
    private int count = 0;
  /**
   * Creates a new DriveToDistance.
   */
  public DriveToDistanceSlowly(DriveBase drive, double inches) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVE_P, 0, Constants.DRIVE_D),
        // This should return the measurement
        () -> drive.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> inches * Constants.ROTATIONS_PER_INCH,
        // This uses the output
        output -> {
          // Use the output here
          if(output > 0.25) {
            drive.setRightMotors(0.25);
            drive.setLeftMotors(0.25);
          }
          else if(output < -0.25) {
            drive.setRightMotors(-0.25);
            drive.setLeftMotors(-0.25);
          }
          else {
            drive.setRightMotors(output);
            drive.setLeftMotors(output);
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(Constants.DISTANCE_TOLERANCE * Constants.ROTATIONS_PER_INCH);
  }

  @Override
  public void initialize() {
    drive.resetPosition();
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
