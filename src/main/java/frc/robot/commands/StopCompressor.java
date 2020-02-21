/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class StopCompressor extends CommandBase {
  private Pneumatics pneumatics;

  /**
   * Creates a new StopCompressor.
   */
  public StopCompressor(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pneumatics = pneumatics;
    addRequirements(this.pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumatics.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
