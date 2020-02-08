/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;


public class RotationControl extends CommandBase {
  private ColorWheel colorWheel;
  private int colorCounts;
  private Joystick operatorController;

  public RotationControl(ColorWheel colorWheel, int colorCounts, Joystick operatorController) {
    this.colorWheel = colorWheel;
    this.colorCounts = colorCounts;
    this.operatorController = operatorController;
    addRequirements(colorWheel);
  }

  @Override
  public void initialize() {
    colorWheel.resetColorCount();
    colorWheel.resetNewColors();
  }

  @Override
  public void execute() {
    colorWheel.turnWheel();
    operatorController.setRumble(RumbleType.kLeftRumble, colorWheel.vibrationControl(colorWheel.getColorCount()));
    SmartDashboard.putNumber("Vibration Power", colorWheel.vibrationControl(colorWheel.getColorCount()));
  }

  @Override
  public void end(boolean interrupted) {
    colorWheel.stopWheel();
  }

  @Override
  public boolean isFinished() {
    boolean requiredRotations = colorWheel.getColorCount() >= colorCounts ? true : false;
    SmartDashboard.putBoolean("Rotations Control Complete", requiredRotations);
    if (requiredRotations == true){
      operatorController.setRumble(RumbleType.kLeftRumble, 0);
    }
    return requiredRotations;
  }
}