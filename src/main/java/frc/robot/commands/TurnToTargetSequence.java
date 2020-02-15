/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drive2019;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToTargetSequence extends SequentialCommandGroup {
  /**
   * Creates a new TurnToTargetSequence.
   */
  public TurnToTargetSequence(Drive2019 drive, Vision vision) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new VisionLightOn(vision),
      new WaitCommand(0.25),
      new TurnToTarget(drive, vision),
      new VisionLightOff(vision)
    );
  }
}
