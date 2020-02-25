/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
    leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
    navx = new AHRS(SerialPort.Port.kMXP);

    leftAlternateEncoder = leftBack.getAlternateEncoder(kAltEncType, kCPR);
    rightAlternateEncoder = rightBack.getAlternateEncoder(kAltEncType, kCPR);
    rightAlternateEncoder.setInverted(true);
    leftAlternateEncoder.setPosition(0);
    rightAlternateEncoder.setPosition(0);
    driveBaseTab.addNumber("Angle", () -> getAngle());
    driveBaseTab.addNumber("Heading", () -> getHeading());
    driveBaseTab.addNumber("Position", () -> getPosition());
    driveBaseTab.addNumber("Right Front Output Current", () -> rightFront.getOutputCurrent());
    driveBaseTab.addNumber("Left Front Output Current", () -> leftFront.getOutputCurrent());
    driveBaseTab.addNumber("Right Back Output Current", () -> rightBack.getOutputCurrent());
    driveBaseTab.addNumber("Left Back Output Current", () -> leftBack.getOutputCurrent());

  }

  public void setRightMotors(double power) {
    //rightFront.set(-power);
    //rightBack.set(-power);
  }

  public void setLeftMotors(double power) {
    //leftFront.set(power);
    //leftBack.set(power);
  }

  public double getPosition() {
    double leftEncoderValue = leftAlternateEncoder.getPosition();
    double rightEncoderValue = rightAlternateEncoder.getPosition();
    double averagePosition = (rightEncoderValue + leftEncoderValue) / 2;
    // SmartDashboard.putNumber("LeftEncoder", leftEncoderValue);
    // SmartDashboard.putNumber("RightEncoder", rightEncoderValue);

    return averagePosition;
  }

  public void resetPosition() {
    leftAlternateEncoder.setPosition(0);
    rightAlternateEncoder.setPosition(0);
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
