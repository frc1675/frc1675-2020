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
  private String targetColor = "Unknown";
  private String theirDetectedColor = "Unknown";
  private String anticipatedColor = "Unknown";
  private String readColor = "Unknown";

  public ColorWheel() {
    spinMotor = new VictorSPX(Constants.WHEEL_MOTOR);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
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

  public int getColorCount() {;
     return colorTransitions;
  }

  public void resetColorCount() {
    colorTransitions = 0;
    transitionColor = currentColor;
  }

  public String getColor() {
    return currentColor;
  }
 
  public String getTheirColor() {
    return theirDetectedColor;
  }

  public String getTargetColor(){
    return targetColor;
  }

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      readColor = "Blue";
      if (anticipatedColor == "Unknown"){
        anticipatedColor = "Yellow";
      }
    } else if (match.color == kRedTarget) {
      readColor = "Red";
      if (anticipatedColor == "Unknown"){
        anticipatedColor = "Green";
      }
    } else if (match.color == kGreenTarget) {
      readColor = "Green";
      if (anticipatedColor == "Unknown"){
        anticipatedColor = "Blue";
      }
    } else if (match.color == kYellowTarget) {
      readColor = "Yellow";
      if (anticipatedColor == "Unknown"){
        anticipatedColor = "Red";
      }
    } else {
      readColor = "Unknown";
    }

    if (readColor == "Blue" && readColor == anticipatedColor) {
      currentColor = "Blue";
      anticipatedColor = "Yellow";
      theirDetectedColor = "Red";
    } else if (readColor == "Red" && readColor == anticipatedColor) {
      currentColor = "Red";
      anticipatedColor = "Green";
      theirDetectedColor = "Blue";
    } else if (readColor == "Green" && readColor == anticipatedColor) {
      currentColor = "Green";
      anticipatedColor = "Blue";
      theirDetectedColor = "Yellow";
    } else if (readColor == "Yellow" && readColor == anticipatedColor) {
      currentColor = "Yellow";
      anticipatedColor = "Red";
      theirDetectedColor = "Green";
    } /*else {
      currentColor = "Unknown";
      theirDetectedColor = "Unknown";
    }*/
    System.out.println("Read Color: "+readColor);

    // System.out.println("Current color: "+currentColor);

    if (currentColor != transitionColor) {
      colorTransitions = colorTransitions + 1;
      transitionColor = currentColor;
      System.out.println("CurrentColor: " +currentColor);
    }

    SmartDashboard.putString("Current Color", currentColor);
    SmartDashboard.putString("read Color", readColor);
    SmartDashboard.putString("anticipated Color", anticipatedColor);
    SmartDashboard.putString("Transition Color", transitionColor);
    SmartDashboard.putNumber("Color Count", colorTransitions);
    SmartDashboard.putString("Competition Color", theirDetectedColor);
    SmartDashboard.putString("Target Color", targetColor);
  }
}