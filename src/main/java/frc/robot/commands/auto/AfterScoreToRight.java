/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AfterScoreToRight extends SequentialCommandGroup {

  private static final double SEGMENT_1 = -25.8;

  private static final double TURN_1_ANGLE = -41.0;
  
  private static final double SEGMENT_2 = -101.9;

  private static final double TURN_2_ANGLE = -139.0;

  
  /**
   * Creates a new MoveToRight.
   */
  public AfterScoreToRight(DriveBase drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveToDistance(drive, SEGMENT_1).withTimeout(2),
      new TurnToAngle(drive, TURN_1_ANGLE).withTimeout(1.5),  
      new DriveToDistance(drive, SEGMENT_2).withTimeout(4),
      new TurnToAngle(drive, TURN_2_ANGLE).withTimeout(2)
    );
  }
}
