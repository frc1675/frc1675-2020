/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive2019 extends SubsystemBase {
  public VictorSPX leftFront;
  public VictorSPX leftBack;
  public VictorSPX rightFront;
  public VictorSPX rightBack;
  public TalonSRX leftMiddle;
  public TalonSRX rightMiddle;
  public int currentPosition;
  public double power;
  public AHRS navx;
  private ShuffleboardTab drive2019Tab = Shuffleboard.getTab("Drive2019");

  public Drive2019() {
    rightMiddle = new TalonSRX(Constants.RIGHT_MIDDLE);
    leftMiddle = new TalonSRX(Constants.LEFT_MIDDLE);
    rightBack = new VictorSPX(Constants.RIGHT_BACK);
    rightFront = new VictorSPX(Constants.RIGHT_FRONT);
    leftBack = new VictorSPX(Constants.LEFT_BACK);
    leftFront = new VictorSPX(Constants.LEFT_FRONT);
    navx = new AHRS(SerialPort.Port.kMXP);

    leftMiddle.setSensorPhase(false);
    rightMiddle.setSensorPhase(false);

    drive2019Tab.addNumber("Current Position", () -> getPosition());
    drive2019Tab.addNumber("Angle", () -> getAngle());
    drive2019Tab.addNumber("Heading", () -> getHeading());
    drive2019Tab.addNumber("Right encoder", () -> rightMiddle.getSelectedSensorPosition());
    drive2019Tab.addNumber("Left encoder", () -> leftMiddle.getSelectedSensorPosition());

  }

  public void setRightMotors(double power) {
    rightFront.set(ControlMode.PercentOutput, -power);
    rightMiddle.set(ControlMode.PercentOutput, -power);
    rightBack.set(ControlMode.PercentOutput, -power);
  }

  public void setLeftMotors(double power) {
    leftFront.set(ControlMode.PercentOutput, power);
    leftMiddle.set(ControlMode.PercentOutput, power);
    leftBack.set(ControlMode.PercentOutput, power);
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public double getHeading() {
    double angle = getAngle();
    double heading = (angle % 360);
    return heading;
  }

  public int getPosition() {
    //int rightPosition = rightMiddle.getSelectedSensorPosition();
    int leftPosition = leftMiddle.getSelectedSensorPosition();
    int averagePosition = leftPosition;

    // currentPosition = currentPosition + 1;
    // return currentPosition;
    return averagePosition;
  }

  public void resetPosition() {
    rightMiddle.setSelectedSensorPosition(0);
    leftMiddle.setSelectedSensorPosition(0);
    currentPosition = 0;
  }

  public void resetAngle() {
    navx.reset();
  }

  @Override
  public void periodic() {
    double position = getPosition();

  }
}
