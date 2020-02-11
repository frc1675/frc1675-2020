/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorWheel;

public class RotationControl extends CommandBase {
  private ColorWheel colorWheel;
  private int colorCounts;
  private Joystick operatorController;
  private ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Color Wheel");
  private NetworkTableEntry rotationComplete;

  public RotationControl(ColorWheel colorWheel, int colorCounts, Joystick operatorController) {
    this.colorWheel = colorWheel;
    this.colorCounts = colorCounts;
    this.operatorController = operatorController;
    addRequirements(colorWheel);
    rotationComplete = colorWheelTab.add("Rotation Complete", false).getEntry();
  }

  public double vibrationPower(int colorTransitions) {
    double vibrationPower = ((double) colorTransitions / Constants.ROTATION_COUNTS_NEEDED - 1) * -1;
    return vibrationPower;
  }

  @Override
  public void initialize() {
    colorWheel.resetColorState();
  }

  @Override
  public void execute() {
    colorWheel.turnWheel();
    operatorController.setRumble(RumbleType.kLeftRumble, vibrationPower(colorWheel.getColorCount()));
    SmartDashboard.putNumber("Vibration Power", vibrationPower(colorWheel.getColorCount()));
  }

  @Override
  public void end(boolean interrupted) {
    colorWheel.stopWheel();
  }

  @Override
  public boolean isFinished() {
    boolean requiredRotations = colorWheel.getColorCount() >= colorCounts ? true : false;
    if (requiredRotations == true) {
      operatorController.setRumble(RumbleType.kLeftRumble, 0);
    }
    rotationComplete.setBoolean(requiredRotations);
    return requiredRotations;
  }
}