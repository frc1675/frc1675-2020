/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

public class PositionControl extends CommandBase {
  private ColorWheel colorWheel;
  private String targetColor;
  private ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Color Wheel");
  private NetworkTableEntry targetColorMatch;

  public PositionControl(ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    addRequirements(colorWheel);
    targetColorMatch = colorWheelTab.add("Rotation Complete", false).getEntry();
  }

  @Override
  public void initialize() {
    targetColor = colorWheel.getTargetColor();
    colorWheel.resetColorState();
  }

  @Override
  public void execute() {
    colorWheel.turnWheel();
  }

  @Override
  public void end(boolean interrupted) {
    colorWheel.stopWheel();
  }

  @Override
  public boolean isFinished() {
    if (targetColor == "Unknown") {
      return true;
    }
    boolean colorMatch = targetColor == colorWheel.getWheelColor() ? true : false;
    targetColorMatch.setBoolean(colorMatch);
    return colorMatch;
  }
}
