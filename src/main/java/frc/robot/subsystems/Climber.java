/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotorRight;
  private CANSparkMax climberMotorLeft;
  private CANEncoder rightEncoder;
  private CANEncoder leftEncoder;
  private Solenoid retractSolenoid;
  private Solenoid releaseSolenoid;

  private boolean climberExtended = false;
  // **
  // * Creates a new Climber.
  // *
private ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");

  public Climber() {
    climberMotorRight = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    climberMotorLeft = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    retractSolenoid = new Solenoid(Constants.CLIMBER_RETRACT_SOLENOID);
    releaseSolenoid = new Solenoid(Constants.CLIMBER_RELEASE_SOLENOID);
    climberMotorLeft.setInverted(true);
    climberMotorRight.setInverted(true);
    retractSolenoid.set(false);
    releaseSolenoid.set(false);
    rightEncoder = climberMotorRight.getEncoder();
    leftEncoder = climberMotorLeft.getEncoder();
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    climberTab.addNumber("Right Climber Encoder Position", () -> rightEncoder.getPosition());
    climberTab.addNumber("Left Climber Encoder Position", () -> leftEncoder.getPosition());
  }

  public void release(){
    releaseSolenoid.set(true);
    climberExtended = true;
  }

  public void engage(){
    retractSolenoid.set(false);
    releaseSolenoid.set(false);
  }

  public void disengage(){
    retractSolenoid.set(true);
  }

  public double getEncoderAverage() {
    double rightEncoderValue = rightEncoder.getPosition();
    double leftEncoderValue = leftEncoder.getPosition();
    double encoderAverage = (rightEncoderValue + leftEncoderValue) / 2;
    return encoderAverage;
  }

  public void pullUp(){
    if (climberExtended /*&& getEncoderAverage() > Constants.CLIMBER_LIMIT*/) {
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

  public boolean isClimberExtended(){
    return climberExtended;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
