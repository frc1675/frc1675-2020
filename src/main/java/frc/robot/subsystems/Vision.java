/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  static NetworkTable table;

  public static enum Pipeline{
    LIGHT_ON(0),LIGHT_OFF(1);
    double pipeline;
    private Pipeline(double pipeline){
      this.pipeline = pipeline;
}

  /**
   * Creates a new Vision.
 * @return 
   */
  public void Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setPipeline(Vision.Pipeline.LIGHT_OFF);
}

  public double getXOffSet() {
    double xOffSet = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return xOffSet;
}
  
  public double getYOffSet() {
    double yOffSet = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    return yOffSet;
}

public double getTargetArea() {
  double targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  return targetArea;
}
  public Pipeline getPipeline() {
    double pipeline = table.getEntry("pipeline").getDouble(0);
    if(pipeline == 1){
      return Pipeline.LIGHT_ON;
    }
    else{
      return Pipeline.LIGHT_OFF;
    }
}

public void setPipeline(Pipeline pipeline) {
  if(pipeline == Pipeline.LIGHT_OFF){
  table.getEntry("pipeline").setDouble(0);
  }
  else if(pipeline == Pipeline.LIGHT_ON){
    table.getEntry("pipeline").setDouble(1);
  }
}

public boolean hasTarget() {
  double targetb = table.getEntry("tv").getDouble(0);
  boolean hasTarget = false;
  if (targetb == 0) {
    hasTarget = false;
  }
  if (targetb == 1) {
    hasTarget = true;
  }
  return hasTarget;
}

  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putBoolean("Has target?", hasTarget());
    SmartDashboard.putNumber("X offset", getXOffSet());
    SmartDashboard.putNumber("Y offset", getYOffSet());
    SmartDashboard.putNumber("Target area", getTargetArea());
    System.out.print("Is this working?");
    
    Pipeline pipeline = getPipeline();
    if(pipeline == Pipeline.LIGHT_OFF){
      SmartDashboard.getString("Pipeline", "Driver mode on");
      }
      else if(pipeline == Pipeline.LIGHT_ON){
        SmartDashboard.putString("Pipeline", "Target mode on");
      }
    }
    }
  }

