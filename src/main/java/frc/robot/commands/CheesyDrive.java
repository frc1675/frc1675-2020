/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class CheesyDrive extends CommandBase {
  private DriveBase driveBase;
  private DoubleSupplier forwardValue;
  private DoubleSupplier turnValue;
  private double ScalingPower;
  
//  public CheesyDrive(Drive2019 drive, DoubleSupplier forwardValue, DoubleSupplier turnValue, double ScalingPower) {
//     this.driveBase = drive;
//     this.forwardValue = forwardValue;
//     this.turnValue = turnValue;
//     this.ScalingPower = ScalingPower;
//     addRequirements(this.driveBase);
//   }

  public CheesyDrive(DriveBase driveBase, DoubleSupplier forwardValue, DoubleSupplier turnValue, double ScalingPower) {
    this.driveBase = driveBase;
    this.forwardValue = forwardValue;
    this.turnValue = turnValue;
    this.ScalingPower = ScalingPower;
    addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnPower = turnValue.getAsDouble();
    double forwardPower = forwardValue.getAsDouble();
    double rightPower = (1 * forwardPower + -1 * turnPower);
    double leftPower = (1 * forwardPower + 1 * turnPower);
    driveBase.setLeftMotors(leftPower * ScalingPower);
    driveBase.setRightMotors(rightPower * ScalingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setLeftMotors(0);
    driveBase.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
