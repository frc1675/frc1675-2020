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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive2019 extends SubsystemBase {
  public VictorSPX leftFront;
  public VictorSPX leftBack;
  public VictorSPX rightFront;
  public VictorSPX rightBack;
  public TalonSRX leftMiddle;
  public TalonSRX rightMiddle;
  /**
   * Creates a new Drive.
   */
  public Drive2019() {
    rightMiddle = new TalonSRX(Constants.RIGHT_MIDDLE);
    leftMiddle = new TalonSRX(Constants.LEFT_MIDDLE);
    rightBack = new VictorSPX(Constants.RIGHT_BACK);
    rightFront = new VictorSPX(Constants.RIGHT_FRONT);
    leftBack = new VictorSPX(Constants.LEFT_BACK);
    leftFront = new VictorSPX(Constants.LEFT_FRONT);

  }
  public void setRightMotors(double power){
    rightFront.set(ControlMode.PercentOutput,power);
    rightMiddle.set(ControlMode.PercentOutput,power);
    rightBack.set(ControlMode.PercentOutput,power);
  }
  public void setLeftMotors(double power){
    leftFront.set(ControlMode.PercentOutput,-power);
    leftMiddle.set(ControlMode.PercentOutput,-power);
    leftBack.set(ControlMode.PercentOutput,-power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
