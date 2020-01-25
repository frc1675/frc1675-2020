/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;
  private Solenoid solenoid;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberMotor1 = new CANSparkMax(Constants.CLIMBER_MOTOR1, MotorType.kBrushless);
    climberMotor2 = new CANSparkMax(Constants.CLIMBER_MOTOR2, MotorType.kBrushless);
    solenoid = new Solenoid(Constants.CLIMBER_SOLENOID);
  }

  public void extend(){
    solenoid.set(true);
  }

  public void lock(){
    solenoid.set(false);
  }

  public void pullUp(){
    climberMotor1.set(Constants.CLIMBER_POWER);
    climberMotor2.set(Constants.CLIMBER_POWER);
  }

  public void stop(){
    climberMotor1.set(0);
    climberMotor2.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
