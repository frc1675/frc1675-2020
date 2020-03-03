/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeInAuto extends ParallelDeadlineGroup {
  /**
   * Creates a new IntakeInAuto.
   */
  public IntakeInAuto(DriveBase drive, double distance, Arm arm, Claw claw) {
    // Add your commands in the super() call.  Add the deadline first.
    super(
      new DriveToDistance(drive, distance)
      //new MoveArmToPosition(arm, Constants.INTAKE_POSITION),
      //new Intake(claw)
    );
  }
}
