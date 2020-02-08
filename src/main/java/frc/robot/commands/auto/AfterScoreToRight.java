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
import frc.robot.subsystems.Drive2019;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AfterScoreToRight extends SequentialCommandGroup {

  private static final double SEGMENT_1 = -12.8;

  private static final double TURN_1_ANGLE = -30;
  
  private static final double SEGMENT_2 = -112.2;

  private static final double TURN_2_ANGLE = -150;

  
  /**
   * Creates a new MoveToRight.
   */
  public AfterScoreToRight(Drive2019 drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new TurnToAngle(drive, TURN_1_ANGLE),  
      new DriveToDistance(drive, SEGMENT_1),
      new TurnToAngle(drive, TURN_2_ANGLE),
      new DriveToDistance(drive, SEGMENT_2)
    );
  }
}
