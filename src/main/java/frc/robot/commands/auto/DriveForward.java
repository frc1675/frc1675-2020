/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToDistance;
import frc.robot.subsystems.Drive2019;
import frc.robot.subsystems.DriveBase;

public class DriveForward extends SequentialCommandGroup {
  Drive2019 drive;
  /**
   * Creates a new DriveForward.
   */
  public DriveForward(DriveBase drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveToDistance(drive, 10).withTimeout(3)
    );
  }
}
