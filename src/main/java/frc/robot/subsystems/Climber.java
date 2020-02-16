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
  private CANSparkMax climberMotorRight;
  private CANSparkMax climberMotorLeft;
  private Solenoid solenoid;

  private boolean climberExtended = false;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberMotorRight = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    climberMotorLeft = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    solenoid = new Solenoid(Constants.CLIMBER_SOLENOID);
    climberMotorLeft.setInverted(true);
    climberMotorRight.setInverted(true);
  }

  public void release(){
    solenoid.set(true);
  }

  public void engage(){
    solenoid.set(false);
    climberExtended = true;
  }

  public void pullUp(){
    if (climberExtended) {
      climberMotorRight.set(Constants.CLIMBER_POWER);
      climberMotorLeft.set(Constants.CLIMBER_POWER);
    }
    else {
      climberMotorRight.set(0);
      climberMotorLeft.set(0);
    }
  }

  public void stop(){
    climberMotorRight.set(0);
    climberMotorLeft.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
