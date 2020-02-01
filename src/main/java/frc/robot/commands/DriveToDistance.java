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
import frc.robot.subsystems.Drive2019;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {
    private Drive2019 drive;
  /**
   * Creates a new DriveToDistance.
   */
  public DriveToDistance(Drive2019 drive, double position) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVE_P, 0, 0),
        // This should return the measurement
        () -> drive.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> position,
        // This uses the output
        output -> {
          // Use the output here
          drive.setRightMotors(output);
          drive.setLeftMotors(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10);
  }
  
    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
