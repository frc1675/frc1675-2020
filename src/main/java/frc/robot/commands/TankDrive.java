/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TankDrive extends CommandBase {
  private Drive drive;
  private DoubleSupplier leftYAxis;
  private DoubleSupplier rightYAxis;
  /**
   * Creates a new TankDrive.
   */
  public TankDrive(Drive drive, DoubleSupplier leftYAxis, DoubleSupplier rightYAxis) {
    this.drive = drive;
    this.leftYAxis = leftYAxis;
    this.rightYAxis = rightYAxis;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightPower = rightYAxis.getAsDouble();
    double leftPower = leftYAxis.getAsDouble();
    drive.setLeftMotors(leftPower);
    drive.setRightMotors(rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setRightMotors(0);
    drive.setLeftMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
