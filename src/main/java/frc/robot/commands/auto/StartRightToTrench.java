/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToDistanceSlowly;
import frc.robot.subsystems.PIDDriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StartRightToTrench extends SequentialCommandGroup {
  private static final double SEGMENT_1 = 177.4;
  /**
   * Creates a new TrenchFromRight.
   */
  public StartRightToTrench(PIDDriveBase drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      new DriveToDistanceSlowly(drive, SEGMENT_1).withTimeout(6)
    );
  }
}
