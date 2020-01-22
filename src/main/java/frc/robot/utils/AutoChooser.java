/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Add your docs here.
 */
public class AutoChooser {

    private SendableChooser<StartPosition> startPositionChooser;
    private SendableChooser<Wait> waitChooser;
    private SendableChooser<AfterScoring> afterScoringChooser;
    private SendableChooser<GatherBalls> gatherBallsChooser;

    public enum StartPosition {
        SCORE_RIGHT,
        SCORE_MIDDLE,
        SCORE_LEFT,
        DONT_SCORE //just move backwards
    }

    public enum Wait {
        ZERO_SECONDS,
        ONE_SECOND,
        TWO_SECONDS,
        THREE_SECONDS,
        FOUR_SECONDS,
        FIVE_SECONDS,
        SIX_SECONDS,
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
        startPositionChooser = new SendableChooser<StartPosition>();
        waitChooser = new SendableChooser<Wait>();
        afterScoringChooser = new SendableChooser<AfterScoring>();
        gatherBallsChooser = new SendableChooser<GatherBalls>();

        //NetworkTableEntry startPositionChooser = Shuffleboard.getTab("Auto")
        //.add("Start Position", 1).withWidget(BuiltInWidgets.kComboBoxChooser).getEntry();

        startPositionChooser.addOption("Start right", StartPosition.SCORE_RIGHT);
        startPositionChooser.addOption("Start center", StartPosition.SCORE_MIDDLE);
        startPositionChooser.addOption("Start left", StartPosition.SCORE_LEFT);
        startPositionChooser.addOption("Go backwards", StartPosition.DONT_SCORE);

        waitChooser.addOption("Don't wait", Wait.ZERO_SECONDS);
        waitChooser.addOption("Wait one second", Wait.ONE_SECOND);
        waitChooser.addOption("Wait two seconds", Wait.TWO_SECONDS);
        waitChooser.addOption("Wait three seconds", Wait.THREE_SECONDS);
        waitChooser.addOption("Wait four seconds", Wait.FOUR_SECONDS);
        waitChooser.addOption("Wait five second", Wait.FIVE_SECONDS);
        waitChooser.addOption("Wait six second", Wait.SIX_SECONDS);

        //Shuffleboard.getTab("Auto")
        //.add("After scoring", 2).withWidget(BuiltInWidgets.kComboBoxChooser);
        
        afterScoringChooser.addOption("Go to right", AfterScoring.RIGHT);
        afterScoringChooser.addOption("Go to center", AfterScoring.MIDDLE);
        afterScoringChooser.addOption("Go to left", AfterScoring.LEFT);

        //Shuffleboard.getTab("Auto")
        //.add("Gather balls", 3).withWidget(BuiltInWidgets.kComboBoxChooser);
        
        gatherBallsChooser.addOption("Trench", GatherBalls.TRENCH);
        gatherBallsChooser.addOption("Shield gernerator", GatherBalls.SHIELD_GENERATOR);
        gatherBallsChooser.addOption("Loading station", GatherBalls.LOADING_STATION);
        gatherBallsChooser.addOption("Nothing", GatherBalls.NOTHING);
    }
    
    public SequentialCommandGroup GenerateAuto() {
        SequentialCommandGroup auto = new SequentialCommandGroup();

        StartPosition selectedStart = (StartPosition) startPositionChooser.getSelected();

        switch(selectedStart) {
            case SCORE_RIGHT:         

            break;

            case SCORE_MIDDLE:

            break;

            case SCORE_LEFT:

            break;

            case DONT_SCORE:

            break;

            default:

            break;

        }

        AfterScoring selectedPosition = (AfterScoring) afterScoringChooser.getSelected();

        switch(selectedPosition) {
            case RIGHT:

            break;

            case MIDDLE:

            break;

            case LEFT:

            break;

        }

        GatherBalls gatherBalls = (GatherBalls) gatherBallsChooser.getSelected();

        switch(gatherBalls) {
            case TRENCH:

            break;

            case SHIELD_GENERATOR:

            break;

            case LOADING_STATION:

            break;

            case NOTHING:

            break;

        }

        return auto;
    }
}
