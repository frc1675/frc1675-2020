/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.lang.Math; 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class CheesyDrive extends CommandBase {
  private DriveBase driveBase;
  private DoubleSupplier forwardValue;
  private DoubleSupplier turnValue;
  private double scalingPower;
  private double prev_fwd_value;
  private double prev_turn_value;
  private final double max_velocity = 10000000000000000000.0;  
  private final double scale_velocity = 0.7;
  private final double sample_dt = 0.01;
  
//  public CheesyDrive(Drive2019 drive, DoubleSupplier forwardValue, DoubleSupplier turnValue, double ScalingPower) {
//     this.driveBase = drive;
//     this.forwardValue = forwardValue;
//     this.turnValue = turnValue;
//     this.scalingPower = ScalingPower;
//     addRequirements(this.driveBase);
//   }

  public CheesyDrive(DriveBase driveBase, DoubleSupplier forwardValue, DoubleSupplier turnValue, double scalingPower) {
    this.driveBase = driveBase;
    this.forwardValue = forwardValue;
    this.prev_fwd_value = forwardValue.getAsDouble();
    this.turnValue = turnValue;
    this.prev_turn_value = turnValue.getAsDouble();
    this.scalingPower = scalingPower;
    addRequirements(this.driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current forward (left Y trigger) and turnValue (right X trigger)
    double turnPower = turnValue.getAsDouble();
    double forwardPower = forwardValue.getAsDouble();    
    // Caclculate the velocity of the triggers
    double fwd_vel = (Math.abs(forwardPower) - this.prev_fwd_value) / sample_dt;
    double turn_vel = (Math.abs(turnPower) - this.prev_turn_value) / sample_dt;
    // Set new previous values
    this.prev_fwd_value = Math.abs(forwardPower);
    this.prev_turn_value = Math.abs(turnPower);
    // Compute Power
    double rightPower = (1 * forwardPower + -1 * turnPower);
    double leftPower = (1 * forwardPower + 1 * turnPower);
    // Compare to set max velocity 
    if(fwd_vel >= max_velocity){
      leftPower = leftPower*scale_velocity;
    }
    if(turn_vel >= max_velocity){
      rightPower = rightPower*scale_velocity; 
    }

    // Send power output values to motors
    driveBase.setLeftMotors(leftPower * scalingPower);
    driveBase.setRightMotors(rightPower * scalingPower);
    System.out.printf("{TurnPower: %.2f/n", max_velocity);
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
