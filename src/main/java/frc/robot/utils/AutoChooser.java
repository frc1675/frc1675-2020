/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.StartLeftToLoadingStation;
import frc.robot.commands.auto.StartMiddleToLoadingStation;
import frc.robot.commands.auto.StartRightToLoadingStation;
import frc.robot.commands.auto.MoveBackward;
import frc.robot.commands.auto.MoveToLeft;
import frc.robot.commands.auto.MoveToMiddle;
import frc.robot.commands.auto.MoveToRight;
import frc.robot.commands.auto.StartLeftToScore;
import frc.robot.commands.auto.StartMiddleToScore;
import frc.robot.commands.auto.StartRightToScore;
import frc.robot.commands.auto.StartLeftToShieldGenerator;
import frc.robot.commands.auto.StartMiddleToShieldGenerator;
import frc.robot.commands.auto.StartRightToShieldGenerator;
import frc.robot.commands.auto.StartLeftToTrench;
import frc.robot.commands.auto.StartMiddleToTrench;
import frc.robot.commands.auto.StartRightToTrench;

/**
 * Add your docs here.
 */
public class AutoChooser {

    private SendableChooser<StartPosition> startPositionChooser;
    private SendableChooser<WaitTime> waitChooser;
    private SendableChooser<AfterScoring> afterScoringChooser;
    private SendableChooser<GatherBalls> gatherBallsChooser;

    public enum WaitTime {
        ZERO_SECONDS,
        ONE_SECOND,
        TWO_SECONDS,
        THREE_SECONDS,
        FOUR_SECONDS,
        FIVE_SECONDS,
        SIX_SECONDS,
        SEVEN_SECONDS,
        EIGHT_SECONDS,
        NINE_SECONDS,
        TEN_SECONDS
    }

    public enum StartPosition {
        SCORE_FROM_RIGHT,
        SCORE_FROM_MIDDLE,
        SCORE_FROM_LEFT,
        DONT_SCORE,
        DRIVE_FORWARD
    }

    public enum AfterScoring {
        RIGHT,
        MIDDLE,
        LEFT
    }

    public enum GatherBalls {
        TRENCH,
        SHIELD_GENERATOR,
        LOADING_STATION,
        NOTHING
    }

    public AutoChooser() {

        //make choosers on smartdashboard
        waitChooser = new SendableChooser<WaitTime>();
        startPositionChooser = new SendableChooser<StartPosition>();
        afterScoringChooser = new SendableChooser<AfterScoring>();
        gatherBallsChooser = new SendableChooser<GatherBalls>();

        waitChooser.addOption("Don't wait", WaitTime.ZERO_SECONDS);
        waitChooser.addOption("Wait one second", WaitTime.ONE_SECOND);
        waitChooser.addOption("Wait two seconds", WaitTime.TWO_SECONDS);
        waitChooser.addOption("Wait three seconds", WaitTime.THREE_SECONDS);
        waitChooser.addOption("Wait four seconds", WaitTime.FOUR_SECONDS);
        waitChooser.addOption("Wait five seconds", WaitTime.FIVE_SECONDS);
        waitChooser.addOption("Wait six seconds", WaitTime.SIX_SECONDS);
        waitChooser.addOption("Wait seven seconds", WaitTime.SEVEN_SECONDS);
        waitChooser.addOption("Wait eight seconds", WaitTime.EIGHT_SECONDS);
        waitChooser.addOption("Wait nine seconds", WaitTime.NINE_SECONDS);
        waitChooser.addOption("Wait ten seconds", WaitTime.TEN_SECONDS);

        startPositionChooser.addOption("Start right", StartPosition.SCORE_FROM_RIGHT);
        startPositionChooser.addOption("Start center", StartPosition.SCORE_FROM_MIDDLE);
        startPositionChooser.addOption("Start left", StartPosition.SCORE_FROM_LEFT);
        startPositionChooser.addOption("Go backwards", StartPosition.DONT_SCORE);
        startPositionChooser.addOption("Test driving forward", StartPosition.DRIVE_FORWARD);

        afterScoringChooser.addOption("Go to right", AfterScoring.RIGHT);
        afterScoringChooser.addOption("Go to center", AfterScoring.MIDDLE);
        afterScoringChooser.addOption("Go to left", AfterScoring.LEFT);

        gatherBallsChooser.addOption("Trench", GatherBalls.TRENCH);
        gatherBallsChooser.addOption("Shield gernerator", GatherBalls.SHIELD_GENERATOR);
        gatherBallsChooser.addOption("Loading station", GatherBalls.LOADING_STATION);
        gatherBallsChooser.addOption("Nothing", GatherBalls.NOTHING);

        SmartDashboard.putData("Wait", waitChooser);
        SmartDashboard.putData("Start", startPositionChooser);
        SmartDashboard.putData("After scoring", afterScoringChooser);
        SmartDashboard.putData("Gather balls", gatherBallsChooser);
    }

    public SequentialCommandGroup GenerateAuto() {
        SequentialCommandGroup auto = new SequentialCommandGroup();

        WaitTime waitTime = (WaitTime) waitChooser.getSelected();

        switch (waitTime) {
        case ZERO_SECONDS:

            break;

        case ONE_SECOND:
            auto.addCommands(new WaitCommand(1));
            break;

        case TWO_SECONDS:
            auto.addCommands(new WaitCommand(2));
            break;

        case THREE_SECONDS:
            auto.addCommands(new WaitCommand(3));
            break;

        case FOUR_SECONDS:
            auto.addCommands(new WaitCommand(4));
            break;

        case FIVE_SECONDS:
            auto.addCommands(new WaitCommand(5));
            break;

        case SIX_SECONDS:
            auto.addCommands(new WaitCommand(6));
            break;

        case SEVEN_SECONDS:
            auto.addCommands(new WaitCommand(7));
            break;

        case EIGHT_SECONDS:
            auto.addCommands(new WaitCommand(8));
            break;

        case NINE_SECONDS:
            auto.addCommands(new WaitCommand(9));
            break;

        case TEN_SECONDS:
            auto.addCommands(new WaitCommand(10));
            break;

        default:

            break;

        }

        StartPosition selectedStart = (StartPosition) startPositionChooser.getSelected();

        switch (selectedStart) {
            case SCORE_FROM_RIGHT:
                auto.addCommands(new StartRightToScore());
                break;

            case SCORE_FROM_MIDDLE:
                auto.addCommands(new StartMiddleToScore());
                break;

            case SCORE_FROM_LEFT:
                auto.addCommands(new StartLeftToScore());
                break;

            case DONT_SCORE:
                auto.addCommands(new MoveBackward());
                return auto;

            case DRIVE_FORWARD:
                auto.addCommands(new DriveForward());

            default:

                break;

        }

        AfterScoring selectedPosition = (AfterScoring) afterScoringChooser.getSelected();
        GatherBalls gatherBalls = (GatherBalls) gatherBallsChooser.getSelected();

        switch(selectedPosition) {
            case RIGHT:
                auto.addCommands(new MoveToRight());
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartRightToTrench());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartRightToShieldGenerator());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new StartRightToLoadingStation());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

            case MIDDLE:
                auto.addCommands(new MoveToMiddle());
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartMiddleToTrench());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartMiddleToShieldGenerator());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new StartMiddleToLoadingStation());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

            case LEFT:
                auto.addCommands(new MoveToLeft());
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartLeftToTrench());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartLeftToShieldGenerator());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new StartLeftToLoadingStation());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

        }

        return auto;
    }
}
