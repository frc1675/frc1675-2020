/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public TalonSRX leftMiddle;
  public TalonSRX rightMiddle;
  private ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  // private CANSparkMax armMotorLeft;
  // private CANSparkMax armMotorRight;
  // private Solenoid solenoid;
  /**
   * Creates a new Arm.
   */
  public Arm() {
    rightMiddle = new TalonSRX(Constants.RIGHT_MIDDLE);
    leftMiddle = new TalonSRX(Constants.LEFT_MIDDLE);
    // armMotorLeft = new CANSparkMax(Constants.ARM_MOTOR_LEFT,
    // MotorType.kBrushless);
    // armMotorRight = new CANSparkMax(Constants.ARM_MOTOR_RIGHT,
    // MotorType.kBrushless);
    // solenoid = new Solenoid(Constants.ARM_SOLENOID);
    rightMiddle.setSensorPhase(true);
    leftMiddle.setSensorPhase(true);
    armTab.addNumber("Position", () -> getPosition());
  }

  public void moveArm(double power) {
    rightMiddle.set(ControlMode.PercentOutput, power);
    leftMiddle.set(ControlMode.PercentOutput, -power);
    // armMotorLeft.set(power);
    // armMotorRight.set(power);
  }

  public void lock() {
    // solenoid.set(true);
    System.out.println("arm lock");
  }

  public void unlock() {
    // solenoid.set(false);
    System.out.println("arm unlock");
  }

  public int getPosition() {
    int rightPosition = rightMiddle.getSelectedSensorPosition();
    int leftPosition = leftMiddle.getSelectedSensorPosition();
    int averagePosition = (rightPosition + leftPosition) / 2;
    return averagePosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}