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
  private Drive2019 Drive2019; 
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(Drive2019 Drive2019) {
    super(
        // The controller that the command will use
        new PIDController(17, 0, 0),
        // This should return the measurement
        () -> Drive2019.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 90 + Drive2019.getAngle(),
        // This uses the output
        output -> {
          Drive2019.setRightMotors(output);
          Drive2019.setLeftMotors(-output);
          // Use the output here
        });
        addRequirements(this.Drive2019);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
