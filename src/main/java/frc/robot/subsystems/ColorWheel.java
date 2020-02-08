/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorWheel extends SubsystemBase {
  private VictorSPX spinMotor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private String transitionColor = "Unknown";
  private int colorTransitions = 0;
  private String currentColor = "Unknown";
  private ShuffleboardTab colorWheelTab = Shuffleboard.getTab("Color Wheel");

  public ColorWheel() {
    spinMotor = new VictorSPX(Constants.WHEEL_MOTOR);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    colorWheelTab.addString("Detected Color", () -> currentColor);
    colorWheelTab.addString("Transition Color", () -> transitionColor);
    colorWheelTab.addNumber("Color Count", () -> colorTransitions);
  }

  public void turnWheel() {
    spinMotor.set(ControlMode.PercentOutput, Constants.COLOR_WHEEL_SPIN_SPEED);
  }

  public void reverseWheel() {
    spinMotor.set(ControlMode.PercentOutput, Constants.REVERSE_COLOR_WHEEL_SPIN_SPEED);
  }

  public void stopWheel() {
    spinMotor.set(ControlMode.PercentOutput, 0);
  }

  public int getColorCount() {
    return colorTransitions;
  }

  public void resetColorCount() {
    colorTransitions = 0;
    transitionColor = currentColor;
  }

  public String getColor() {
    return currentColor;
  }

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      currentColor = "Blue";
    } else if (match.color == kRedTarget) {
      currentColor = "Red";
    } else if (match.color == kGreenTarget) {
      currentColor = "Green";
    } else if (match.color == kYellowTarget) {
      currentColor = "Yellow";
    } else {
      currentColor = "Unknown";
    }

    if (currentColor != transitionColor) {
      colorTransitions = colorTransitions + 1;
      transitionColor = currentColor;
    }
  }
}