/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax ArmMotorLeft;
  private CANSparkMax ArmMotorRight;
  private Solenoid solenoid;
  /**
   * Creates a new Arm.
   */
  public Arm() {
    ArmMotorLeft = new CANSparkMax(Constants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    ArmMotorRight = new CANSparkMax(Constants.ARM_Motor_RIGHT, MotorType.kBrushless);
    solenoid = new Solenoid(Constants.SOLENOID);
  }
  public void moveArm(double power){
    ArmMotorLeft.set(power);
    ArmMotorRight.set(power);
  }
  public void lock(){
    solenoid.set(true);
  }
  public void unlock(){
    solenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
