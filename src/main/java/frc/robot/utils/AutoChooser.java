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
import frc.robot.commands.auto.LoadingFromLeft;
import frc.robot.commands.auto.LoadingFromMiddle;
import frc.robot.commands.auto.LoadingFromRight;
import frc.robot.commands.auto.MoveBackward;
import frc.robot.commands.auto.MoveToLeft;
import frc.robot.commands.auto.MoveToMiddle;
import frc.robot.commands.auto.MoveToRight;
import frc.robot.commands.auto.ScoreFromLeft;
import frc.robot.commands.auto.ScoreFromMiddle;
import frc.robot.commands.auto.ScoreFromRight;
import frc.robot.commands.auto.ShieldFromLeft;
import frc.robot.commands.auto.ShieldFromMiddle;
import frc.robot.commands.auto.ShieldFromRight;
import frc.robot.commands.auto.TrenchFromLeft;
import frc.robot.commands.auto.TrenchFromMiddle;
import frc.robot.commands.auto.TrenchFromRight;
import frc.robot.commands.auto.Wait;

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
    }

    public enum StartPosition {
        SCORE_FROM_RIGHT,
        SCORE_FROM_MIDDLE,
        SCORE_FROM_LEFT,
        DONT_SCORE //just move backwards
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

        startPositionChooser.addOption("Start right", StartPosition.SCORE_FROM_RIGHT);
        startPositionChooser.addOption("Start center", StartPosition.SCORE_FROM_MIDDLE);
        startPositionChooser.addOption("Start left", StartPosition.SCORE_FROM_LEFT);
        startPositionChooser.addOption("Go backwards", StartPosition.DONT_SCORE);

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
            System.out.println("Start waiting(1)");
            auto.addCommands(new Wait(1));
            System.out.println("End waiting");
            break;

        case TWO_SECONDS:
            System.out.println("Start waiting(2)");
            auto.addCommands(new Wait(2));
            System.out.println("End waiting");
            break;

        case THREE_SECONDS:
            System.out.println("Start waiting(3)");
            auto.addCommands(new Wait(3));
            System.out.println("End waiting");
            break;

        case FOUR_SECONDS:
            System.out.println("Start waiting(4)");
            auto.addCommands(new Wait(4));
            System.out.println("End waiting");
            break;

        case FIVE_SECONDS:
            System.out.println("Start waiting(5)");
            auto.addCommands(new Wait(5));
            System.out.println("End waiting");
            break;

        case SIX_SECONDS:
            System.out.println("Start waiting(6)");
            auto.addCommands(new Wait(6));
            System.out.println("End waiting");
            break;

        default:

            break;

        }

        StartPosition selectedStart = (StartPosition) startPositionChooser.getSelected();

        switch (selectedStart) {
            case SCORE_FROM_RIGHT:
                auto.addCommands(new ScoreFromRight());
                break;

            case SCORE_FROM_MIDDLE:
                auto.addCommands(new ScoreFromMiddle());
                break;

            case SCORE_FROM_LEFT:
                auto.addCommands(new ScoreFromLeft());
                break;

            case DONT_SCORE:
                auto.addCommands(new MoveBackward());
                return auto;

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
                        auto.addCommands(new TrenchFromRight());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new ShieldFromRight());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new LoadingFromRight());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

            case MIDDLE:
                auto.addCommands(new MoveToMiddle());
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new TrenchFromMiddle());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new ShieldFromMiddle());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new LoadingFromMiddle());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

            case LEFT:
                auto.addCommands(new MoveToLeft());
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new TrenchFromLeft());
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new ShieldFromLeft());
                        break;
        
                    case LOADING_STATION:
                        auto.addCommands(new LoadingFromLeft());
                        break;
        
                    case NOTHING:
        
                        break;
        
                }
                break;

        }

        return auto;
    }
}
