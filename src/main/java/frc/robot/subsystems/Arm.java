/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private CANSparkMax armMotorLeft;
  private CANSparkMax armMotorRight;
  private Solenoid solenoid;
  private DutyCycleEncoder encoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;

  /**
   * Creates a new Arm.
   */
  public Arm() {
    armMotorLeft = new CANSparkMax(Constants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(Constants.ARM_MOTOR_RIGHT, MotorType.kBrushless);
    solenoid = new Solenoid(Constants.ARM_SOLENOID);

    encoder = new DutyCycleEncoder(0);

    armMotorRight.setInverted(true);

    armTab.addNumber("Position", () -> getPosition());
    armTab.addBoolean("Connected?", () -> encoder.isConnected());
    armTab.addNumber("Frequency", () -> encoder.getFrequency());
    encoder.setDistancePerRotation(360);

  }

  public void moveArm(double power) {
    armMotorLeft.set(power);
    armMotorRight.set(power);
  }

  public void lock() {
    solenoid.set(true);
  }

  public void unlock() {
    solenoid.set(false);
  }

  public double getPosition() {
    double armEncoderValue = encoder.getDistance();
    return armEncoderValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}