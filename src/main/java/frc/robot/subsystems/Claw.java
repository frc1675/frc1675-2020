/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  public VictorSPX intakeMotor;
  public VictorSPX outtakeMotor;
  /**
   * Creates a new Claw.
   */
  public Claw() {
    intakeMotor = new VictorSPX(Constants.INTAKE_POWER);
    outtakeMotor = new VictorSPX(Constants.OUTPUT_POWER);
  }
  public void intake(){
 
  }
  public void output(){

  }
  public void stop(){
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
