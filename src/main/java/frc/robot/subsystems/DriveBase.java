/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;
  private WPI_TalonSRX leftFrontSim;
  private WPI_TalonSRX leftBackSim;
  private WPI_TalonSRX rightFrontSim;
  private WPI_TalonSRX rightBackSim;
  private CANEncoder leftAlternateEncoder;
  private CANEncoder rightAlternateEncoder;
  private AHRS navx;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;
  private ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drive Base");

  /**
   * Creates a new Drive.
   */
  public DriveBase() {
    if(RobotBase.isReal()){
      rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
      rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
      leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
      leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);

      leftAlternateEncoder = leftBack.getAlternateEncoder(kAltEncType, kCPR);
      rightAlternateEncoder = rightBack.getAlternateEncoder(kAltEncType, kCPR);

      //rightAlternateEncoder.setInverted(true);
      leftAlternateEncoder.setPosition(0);
      rightAlternateEncoder.setPosition(0);

      driveBaseTab.addNumber("Right position", () -> -rightAlternateEncoder.getPosition());
      driveBaseTab.addNumber("Left Position", () -> -leftAlternateEncoder.getPosition());

      driveBaseTab.addNumber("Right Front Output Current", () -> rightFront.getOutputCurrent());
      driveBaseTab.addNumber("Left Front Output Current", () -> leftFront.getOutputCurrent());
      driveBaseTab.addNumber("Right Back Output Current", () -> rightBack.getOutputCurrent());
      driveBaseTab.addNumber("Left Back Output Current", () -> leftBack.getOutputCurrent());
    }
    else {
      rightBackSim = new WPI_TalonSRX(Constants.RIGHT_BACK);
      rightFrontSim = new WPI_TalonSRX(Constants.RIGHT_FRONT);
      leftBackSim = new WPI_TalonSRX(Constants.LEFT_BACK);
      leftFrontSim = new WPI_TalonSRX(Constants.LEFT_FRONT);

      driveBaseTab.addNumber("Right position", () -> -rightBackSim.getSelectedSensorPosition());
      driveBaseTab.addNumber("Left Position", () -> -leftBackSim.getSelectedSensorPosition());
    }
    
    navx = new AHRS(SerialPort.Port.kMXP);
    
    driveBaseTab.addNumber("Angle", () -> getAngle());
    driveBaseTab.addNumber("Heading", () -> getHeading());
    driveBaseTab.addNumber("Position", () -> getPosition());
    
  }

  public void setRightMotors(double power) {
    if(RobotBase.isReal()) {
      rightFront.set(-power);
      rightBack.set(-power);
    }
    else {
      rightFrontSim.set(-power);
      rightBackSim.set(-power);
    }
  }

  public void setLeftMotors(double power) {
    if(RobotBase.isReal()) {
      leftFront.set(-power);
      leftBack.set(-power);
    }
    else {
      leftFrontSim.set(-power);
      leftBackSim.set(-power);
    }
  }

  public double getPosition() {
    double averagePosition = 0;
    double leftEncoderValue = 0;
    double rightEncoderValue = 0;
    if(RobotBase.isReal()) {
      leftEncoderValue = -leftAlternateEncoder.getPosition();
      rightEncoderValue = -rightAlternateEncoder.getPosition();
      // SmartDashboard.putNumber("LeftEncoder", leftEncoderValue);
      // SmartDashboard.putNumber("RightEncoder", rightEncoderValue);
    }
    else {
      leftEncoderValue = -leftBackSim.getSelectedSensorPosition();
      rightEncoderValue = -rightBackSim.getSelectedSensorPosition();
    }
    averagePosition = (rightEncoderValue + leftEncoderValue) / 2;
    return averagePosition;
  }

  public void resetPosition() {
    if(RobotBase.isReal()) {
      leftAlternateEncoder.setPosition(0);
      rightAlternateEncoder.setPosition(0);
    }
    else {
      leftBackSim.setSelectedSensorPosition(0);
      rightBackSim.setSelectedSensorPosition(0);
    }
  }
  
  public void resetAngle() {
    navx.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public double getHeading() {
    double angle = getAngle();
    double heading = (angle % 360);
    return heading;
  }

}
