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

public class Claw extends SubsystemBase {
  public TalonSRX clawMotor;
  /**
   * Creates a new Claw.
   */
  public Claw() {
    clawMotor = new TalonSRX(Constants.CLAW_MOTOR);
  }
  public void intake(){
    clawMotor.set(ControlMode.PercentOutput,Constants.INTAKE_POWER);
  }
  public void output(){
    clawMotor.set(ControlMode.PercentOutput,Constants.OUTPUT_POWER);
  }
  public void stop(){
    clawMotor.set(ControlMode.PercentOutput, 0);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
