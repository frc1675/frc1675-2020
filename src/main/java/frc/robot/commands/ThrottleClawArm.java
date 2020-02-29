/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class ThrottleClawArm extends CommandBase {
  private Claw claw;
  private int timer = 0;

  /**
   * Creates a new ThrottleClawArm.
   */
  public ThrottleClawArm(Claw claw) {
    this.claw = claw;
    addRequirements(this.claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer = timer + 1;
    if (timer <= Constants.THROTTLE_TIMER) {
      claw.intake();
    } else if (timer <= Constants.THROTTLE_TIMER*2) {
      claw.output();
    } else {
      timer = 0;
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
