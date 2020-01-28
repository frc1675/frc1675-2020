/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
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
  private CANSparkMax leftMiddle;
  private CANSparkMax rightMiddle;
  public AHRS navx;

  /**
   * Creates a new Drive.
   */
  public DriveBase() {
    rightMiddle = new CANSparkMax(Constants.RIGHT_MIDDLE, MotorType.kBrushless);
    leftMiddle = new CANSparkMax(Constants.LEFT_MIDDLE, MotorType.kBrushless);
    rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
    leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
    navx = new AHRS(SerialPort.Port.kMXP);
  }

  public void setRightMotors(double power) {
    rightFront.set(power);
    rightMiddle.set(power);
    rightBack.set(power);
  }

  public void setLeftMotors(double power) {
    leftFront.set(-power);
    leftMiddle.set(-power);
    leftBack.set(-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Heading", getHeading());
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
}
