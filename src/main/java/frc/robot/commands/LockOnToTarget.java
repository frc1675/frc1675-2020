/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drive2019;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LockOnToTarget extends PIDCommand {
  private DoubleSupplier forwardValue;
  /**
   * Creates a new LockOnToTarget.
   */
  public LockOnToTarget(Drive2019 drive, DoubleSupplier forwardValue, Vision vision) {
    super(
        // The controller that the command will use
        new PIDController(0.01111, 0, 0),
        // This should return the measurement
        vision::getXOffSet,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        turnPower -> {
          double forwardPower = forwardValue.getAsDouble() * 0.5;
          System.out.println(forwardPower + " Forward value");
          double rightPower = (1 * forwardPower + -1 * turnPower); 
          double leftPower = (1 * forwardPower + 1 * turnPower);
          drive.setLeftMotors(leftPower);
          drive.setRightMotors(rightPower);
          // Use the output here
        });
        addRequirements(drive);
        getController().enableContinuousInput(-180, 180);
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        getController().setTolerance(10);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
