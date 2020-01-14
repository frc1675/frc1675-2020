/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  Solenoid piston;
  /**
   * Creates a new Grabber.
   */
  public Grabber() {
    piston = new Solenoid(2);
    
  }
  public void release(){
    piston.set(true);
  }
  public void retract(){
    piston.set(false);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
