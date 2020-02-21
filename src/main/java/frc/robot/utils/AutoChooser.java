/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AfterScoreToLeft;
import frc.robot.commands.auto.AfterScoreToMiddle;
import frc.robot.commands.auto.AfterScoreToRight;
import frc.robot.commands.auto.DriveBackward;
import frc.robot.commands.auto.StartLeftToScore;
import frc.robot.commands.auto.StartLeftToShieldGenerator;
import frc.robot.commands.auto.StartLeftToTrench;
import frc.robot.commands.auto.StartMiddleToScore;
import frc.robot.commands.auto.StartMiddleToShieldGenerator;
import frc.robot.commands.auto.StartMiddleToTrench;
import frc.robot.commands.auto.StartRightToScore;
import frc.robot.commands.auto.StartRightToShieldGenerator;
import frc.robot.commands.auto.StartRightToTrench;
import frc.robot.subsystems.Drive2019;

/**
 * Add your docs here.
 */
public class AutoChooser {

    private SendableChooser<StartPosition> startPositionChooser;
    private SendableChooser<AfterScoring> afterScoringChooser;
    private SendableChooser<GatherBalls> gatherBallsChooser;
    
    private ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Station");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    NetworkTableEntry waitSlider;

    private ComplexWidget startWidget;
    private ComplexWidget afterScoringWidget;
    private ComplexWidget gatherBallsWidget;

    private Drive2019 drive;

    private double waitTime;

    public enum StartPosition {
        RIGHT_TO_SCORE, 
        MIDDLE_TO_SCORE, 
        LEFT_TO_SCORE, 
        DRIVE_BACKWARD
    }

    public enum AfterScoring {
        RIGHT, 
        MIDDLE, 
        LEFT, 
        NOTHING
    }

    public enum GatherBalls {
        TRENCH, 
        SHIELD_GENERATOR,
        // LOADING_STATION,
        NOTHING
    }

    public AutoChooser(Drive2019 drive) {
        this.drive = drive;

        // make choosers on smartdashboard
        startPositionChooser = new SendableChooser<StartPosition>();
        afterScoringChooser = new SendableChooser<AfterScoring>();
        gatherBallsChooser = new SendableChooser<GatherBalls>();

        waitSlider = driverTab.add("Wait time", 2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10, "block increment", .5))
            .withSize(2, 1)
            .withPosition(0, 0)
            .getEntry();

        startPositionChooser.addOption("Start right", StartPosition.RIGHT_TO_SCORE);
        startPositionChooser.addOption("Start center", StartPosition.MIDDLE_TO_SCORE);
        startPositionChooser.addOption("Start left", StartPosition.LEFT_TO_SCORE);
        startPositionChooser.addOption("Go backwards", StartPosition.DRIVE_BACKWARD);

        startWidget = driverTab.add("Start psition", startPositionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(0, 1);

        afterScoringChooser.addOption("Go to right", AfterScoring.RIGHT);
        afterScoringChooser.addOption("Go to center", AfterScoring.MIDDLE);
        afterScoringChooser.addOption("Go to left", AfterScoring.LEFT);
        afterScoringChooser.addOption("Do nothing", AfterScoring.NOTHING);

        afterScoringWidget = driverTab.add("After scoring position", afterScoringChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(0, 2);

        gatherBallsChooser.addOption("Trench", GatherBalls.TRENCH);
        gatherBallsChooser.addOption("Shield gernerator", GatherBalls.SHIELD_GENERATOR);
        // gatherBallsChooser.addOption("Loading station", GatherBalls.LOADING_STATION);
        gatherBallsChooser.addOption("Nothing", GatherBalls.NOTHING);

        gatherBallsWidget = driverTab.add("Gather balls location", gatherBallsChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(0, 3);

        SmartDashboard.putData("Start", startPositionChooser);
        SmartDashboard.putData("After scoring", afterScoringChooser);
        SmartDashboard.putData("Gather balls", gatherBallsChooser);
    }

    public SequentialCommandGroup GenerateAuto() {
        SequentialCommandGroup auto = new SequentialCommandGroup();

        waitTime = waitSlider.getDouble(0);

        auto.addCommands(new WaitCommand(waitTime));

        StartPosition selectedStart = (StartPosition) startPositionChooser.getSelected();

        switch (selectedStart) {
        case RIGHT_TO_SCORE:
            auto.addCommands(new StartRightToScore(drive));
            break;

        case MIDDLE_TO_SCORE:
            auto.addCommands(new StartMiddleToScore(drive));
            break;

        case LEFT_TO_SCORE:
            auto.addCommands(new StartLeftToScore(drive));
            break;

        case DRIVE_BACKWARD:
            auto.addCommands(new DriveBackward(drive));
            break;

        default:
            break;

        }

        AfterScoring selectedPosition = (AfterScoring) afterScoringChooser.getSelected();
        GatherBalls gatherBalls = (GatherBalls) gatherBallsChooser.getSelected();

        switch (selectedPosition) {
        case RIGHT:
            auto.addCommands(new AfterScoreToRight(drive));
            switch (gatherBalls) {
            case TRENCH:
                auto.addCommands(new StartRightToTrench(drive));
                break;

            case SHIELD_GENERATOR:
                auto.addCommands(new StartRightToShieldGenerator(drive));
                break;

            /*
             * case LOADING_STATION: auto.addCommands(new StartRightToLoadingStation());
             * break;
             */
            case NOTHING:
                break;

            }
            break;

        case MIDDLE:
            auto.addCommands(new AfterScoreToMiddle(drive));
            switch (gatherBalls) {
            case TRENCH:
                auto.addCommands(new StartMiddleToTrench(drive));
                break;

            case SHIELD_GENERATOR:
                auto.addCommands(new StartMiddleToShieldGenerator(drive));
                break;

            /*
             * case LOADING_STATION: auto.addCommands(new StartMiddleToLoadingStation());
             * break;
             */
            case NOTHING:
                break;

            }
            break;

        case LEFT:
            auto.addCommands(new AfterScoreToLeft(drive));
            switch (gatherBalls) {
            case TRENCH:
                auto.addCommands(new StartLeftToTrench(drive));
                break;

            case SHIELD_GENERATOR:
                auto.addCommands(new StartLeftToShieldGenerator(drive));
                break;

            /*
             * case LOADING_STATION: auto.addCommands(new StartLeftToLoadingStation());
             * break;
             */
            case NOTHING:
                break;

            }
        case NOTHING:
            return auto;

        }
        return auto;
    }
}
