/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotorRight;
  private CANSparkMax climberMotorLeft;
  private WPI_TalonSRX climberMotorRightSim;
  private WPI_TalonSRX climberMotorLeftSim;
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
    if(RobotBase.isReal()) {
      climberMotorRight = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
      climberMotorLeft = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
      rightEncoder = climberMotorRight.getEncoder();
      leftEncoder = climberMotorLeft.getEncoder();
      rightEncoder.setPosition(0);
      leftEncoder.setPosition(0);
      climberTab.addNumber("Right Climber Encoder Position", () -> rightEncoder.getPosition());
      climberTab.addNumber("Left Climber Encoder Position", () -> leftEncoder.getPosition());
      climberMotorLeft.setInverted(true);
      climberMotorRight.setInverted(true);
    }
    else {
      climberMotorRightSim = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_RIGHT);
      climberMotorLeftSim = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_LEFT);
      climberMotorLeftSim.setInverted(true);
      climberMotorRightSim.setInverted(true);

      climberTab.addNumber("Left Climber Encoder Position", () -> climberMotorLeftSim.getSelectedSensorPosition());
    }
    retractSolenoid = new Solenoid(Constants.CLIMBER_RETRACT_SOLENOID);
    releaseSolenoid = new Solenoid(Constants.CLIMBER_RELEASE_SOLENOID);
    
    retractSolenoid.set(false);
    releaseSolenoid.set(false);
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
    double encoderAverage = 0;
    double rightEncoderValue = 0;
    double leftEncoderValue = 0;
    if(RobotBase.isReal()) {
      rightEncoderValue = rightEncoder.getPosition();
      leftEncoderValue = leftEncoder.getPosition();
    }
    encoderAverage = (rightEncoderValue + leftEncoderValue) / 2;
    return encoderAverage;
  }

  public void pullUp(){
    if (climberExtended) {
      if(RobotBase.isReal()) {
        climberMotorRight.set(Constants.CLIMBER_POWER);
        climberMotorLeft.set(Constants.CLIMBER_POWER);
      }
      else {
        climberMotorRightSim.set(Constants.CLIMBER_POWER);
        climberMotorLeftSim.set(Constants.CLIMBER_POWER);
      }
    }
    else {
      if(RobotBase.isReal()) {
        climberMotorRight.set(0);
        climberMotorLeft.set(0);
      }
      else {
        climberMotorRightSim.set(0);
        climberMotorLeftSim.set(0);
      }
    }
  }

  public void stop(){
    if(RobotBase.isReal()) {
      climberMotorRight.set(0);
      climberMotorLeft.set(0);
    }
    else {
      climberMotorRightSim.set(0);
      climberMotorLeftSim.set(0);
    }
  }

  public boolean isClimberExtended(){
    return climberExtended;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
