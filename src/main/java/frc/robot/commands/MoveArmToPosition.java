/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MoveArmToPosition extends PIDCommand {
  private DoubleSupplier armValue;
  private Arm arm;

  /**
   * Creates a new MoveArmToPosition.
   */
  public MoveArmToPosition(Arm arm, DoubleSupplier armValue) {
    super(
        // The controller that the command will use
        new PIDController(0.00278, 0, 0),
        // This should return the measurement
        () -> arm.getPosition(),
        // This should return the setpoint (can also be a constant)
        armValue,
        // This uses the output
        output -> {
          // Use the output here
          arm.moveArm(output);
        });
    this.arm = arm;
    this.armValue = armValue;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1000);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = getController().atSetpoint();
    if (atSetpoint) {
      arm.lock();
    }
    
    return atSetpoint;

  }

}
