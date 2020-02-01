/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive2019;

public class CheesyDrive extends CommandBase {
  private Drive2019 drive;
  private DoubleSupplier forwardValue;
  private DoubleSupplier turnValue;
  
  /**
   * Creates a new CheesyDrive.
   */
  public CheesyDrive(Drive2019 drive, DoubleSupplier forwardValue, DoubleSupplier turnValue) {
    this.drive = drive;
    this.forwardValue = forwardValue;
    this.turnValue = turnValue;
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
    double turnPower = turnValue.getAsDouble();
    double forwardPower = forwardValue.getAsDouble();
    double rightPower = (1 * forwardPower + -1 * turnPower);
    System.out.println(forwardPower);
    double leftPower = (1 * forwardPower + 1 * turnPower);
    drive.setLeftMotors(leftPower);
    drive.setRightMotors(rightPower);
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
