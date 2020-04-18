/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;

public class CheesyDrive extends CommandBase {
  private DriveBase drive;
  private Climber climber;
  private DoubleSupplier forwardValue;
  private DoubleSupplier turnValue;
  private double scalingPower;

  // public CheesyDrive(Drive2019 drive, DoubleSupplier forwardValue,
  // DoubleSupplier turnValue, double ScalingPower) {
  // this.driveBase = drive;
  // this.forwardValue = forwardValue;
  // this.turnValue = turnValue;
  // this.scalingPower = ScalingPower;
  // addRequirements(this.driveBase);
  // }

  public CheesyDrive(DriveBase drive, Climber climber, DoubleSupplier forwardValue, DoubleSupplier turnValue,
      double scalingPower) {
    this.drive = drive;
    this.climber = climber;
    this.forwardValue = forwardValue;
    this.turnValue = turnValue;
    this.scalingPower = scalingPower;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnPower = 0.6 * turnValue.getAsDouble();
    double forwardPower = forwardValue.getAsDouble();
    double rightPower = (1 * forwardPower + -1 * turnPower);
    double leftPower = (1 * forwardPower + 1 * turnPower);
    if (climber.isClimberExtended()) {
      drive.setLeftMotors(leftPower * Constants.CLIMBER_POWER_DRIVE);
      drive.setRightMotors(rightPower * Constants.CLIMBER_POWER_DRIVE);
    } else {
      drive.setLeftMotors(leftPower * scalingPower);
      drive.setRightMotors(rightPower * scalingPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setLeftMotors(0);
    drive.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
