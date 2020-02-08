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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;
  CANEncoder leftAlternateEncoder;
  CANEncoder rightAlternateEncoder;
  // private CANSparkMax leftMiddle;
  // private CANSparkMax rightMiddle;
  public AHRS navx;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;

  /**
   * Creates a new Drive.
   */
  public DriveBase() {
    // rightMiddle = new CANSparkMax(Constants.RIGHT_MIDDLE, MotorType.kBrushless);
    // leftMiddle = new CANSparkMax(Constants.LEFT_MIDDLE, MotorType.kBrushless);
    rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
    leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
    navx = new AHRS(SerialPort.Port.kMXP);

    leftAlternateEncoder = leftFront.getAlternateEncoder(kAltEncType, kCPR);
    rightAlternateEncoder = rightFront.getAlternateEncoder(kAltEncType, kCPR);

  }

  public void setRightMotors(double power) {
    rightFront.set(power);
    // rightMiddle.set(power);
    rightBack.set(power);
  }

  public void setLeftMotors(double power) {
    leftFront.set(-power);
    // leftMiddle.set(-power);
    leftBack.set(-power);
  }

  // public int getPosition(){
  // rightMiddle.getEncoder();
  // int rightPosition = rightMiddle.getPosition();
  // int leftPosition = leftMiddle.getSelectedSensorPosition();
  // int averagePosition = (rightPosition + leftPosition)/2;

  // //currentPosition = currentPosition + 1;
  // //return currentPosition;
  // return averagePosition;
  // }

  // public void resetPosition(){
  // rightMiddle.setSelectedSensorPosition(0);
  // leftMiddle.setSelectedSensorPosition(0);
  // currentPosition = 0;
  // }

  @Override
  public void periodic() {
    double leftEncoderValue = leftAlternateEncoder.getPosition();
    double rightEncoderValue = rightAlternateEncoder.getPosition();
    System.out.println("left: " + leftEncoderValue);
    System.out.println("right: " + rightEncoderValue);
    SmartDashboard.putNumber("Angle", getAngle());
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public double getHeading() {
    double angle = getAngle();
    double heading = (angle % 360);
    System.out.println("Heading =" + heading);
    return heading;
  }

  public void robotInit() {
  }
}
