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
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.MoveBackward;
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
    public ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Station");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    NetworkTableEntry waitSlider;
    private ComplexWidget startWidget;
    private ComplexWidget afterScoringWidget;
    private ComplexWidget gatherBallsWidget;

    private Drive2019 drive;

    private double waitTime;

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
        LEFT,
        NOTHING
    }

    public enum GatherBalls {
        TRENCH,
        SHIELD_GENERATOR,
        //LOADING_STATION,
        NOTHING
    }

    public AutoChooser(Drive2019 drive) {
        this.drive = drive;

        //make choosers on smartdashboard
        startPositionChooser = new SendableChooser<StartPosition>();
        afterScoringChooser = new SendableChooser<AfterScoring>();
        gatherBallsChooser = new SendableChooser<GatherBalls>();

        waitSlider = Shuffleboard.getTab("Autonomous")
            .add("Wait time", 2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10, "block increment", .5))
            .getEntry();

        startPositionChooser.addOption("Start right", StartPosition.SCORE_FROM_RIGHT);
        startPositionChooser.addOption("Start center", StartPosition.SCORE_FROM_MIDDLE);
        startPositionChooser.addOption("Start left", StartPosition.SCORE_FROM_LEFT);
        startPositionChooser.addOption("Go backwards", StartPosition.DONT_SCORE);
        startPositionChooser.addOption("Test driving forward", StartPosition.DRIVE_FORWARD);

        startWidget = autoTab.add("Start psition", startPositionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);

        afterScoringChooser.addOption("Go to right", AfterScoring.RIGHT);
        afterScoringChooser.addOption("Go to center", AfterScoring.MIDDLE);
        afterScoringChooser.addOption("Go to left", AfterScoring.LEFT);
        afterScoringChooser.addOption("Do nothing", AfterScoring.NOTHING);

        afterScoringWidget = autoTab.add("After scoring position", afterScoringChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser); 

        gatherBallsChooser.addOption("Trench", GatherBalls.TRENCH);
        gatherBallsChooser.addOption("Shield gernerator", GatherBalls.SHIELD_GENERATOR);
        //gatherBallsChooser.addOption("Loading station", GatherBalls.LOADING_STATION);
        gatherBallsChooser.addOption("Nothing", GatherBalls.NOTHING);

        gatherBallsWidget = autoTab.add("Gather balls location", gatherBallsChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser); 

        SmartDashboard.putData("Start", startPositionChooser);
        SmartDashboard.putData("After scoring", afterScoringChooser);
        SmartDashboard.putData("Gather balls", gatherBallsChooser);
    }

    public SequentialCommandGroup GenerateAuto() {
        SequentialCommandGroup auto = new SequentialCommandGroup();

        waitTime = waitSlider.getDouble(0);
        System.out.println("Wait time: "+waitTime);
        

        auto.addCommands(new WaitCommand(waitTime));

        StartPosition selectedStart = (StartPosition) startPositionChooser.getSelected();

        switch (selectedStart) {
            case SCORE_FROM_RIGHT:
                auto.addCommands(new StartRightToScore(drive));
                break;

            case SCORE_FROM_MIDDLE:
                auto.addCommands(new StartMiddleToScore(drive));
                break;

            case SCORE_FROM_LEFT:
                auto.addCommands(new StartLeftToScore(drive));
                break;

            case DONT_SCORE:
                auto.addCommands(new MoveBackward(drive));
                return auto;

            case DRIVE_FORWARD:
                auto.addCommands(new DriveForward(drive));
                return auto;
                
            default:

                break;

        }

        AfterScoring selectedPosition = (AfterScoring) afterScoringChooser.getSelected();
        GatherBalls gatherBalls = (GatherBalls) gatherBallsChooser.getSelected();

        switch(selectedPosition) {
            case RIGHT:
                auto.addCommands(new AfterScoreToRight(drive));
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartRightToTrench(drive));
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartRightToShieldGenerator(drive));
                        break;
        
                    /*case LOADING_STATION:
                        auto.addCommands(new StartRightToLoadingStation());
                        break;
                        */
                    case NOTHING:
        
                        break;
        
                }
                break;

            case MIDDLE:
                auto.addCommands(new AfterScoreToMiddle(drive));
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartMiddleToTrench(drive));
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartMiddleToShieldGenerator(drive));
                        break;
        
                    /*case LOADING_STATION:
                        auto.addCommands(new StartMiddleToLoadingStation());
                        break;
                        */
                    case NOTHING:
        
                        break;
        
                }
                break;

            case LEFT:
                auto.addCommands(new AfterScoreToLeft(drive));
                switch(gatherBalls) {
                    case TRENCH:
                        auto.addCommands(new StartLeftToTrench(drive));
                        break;
        
                    case SHIELD_GENERATOR:
                        auto.addCommands(new StartLeftToShieldGenerator(drive));
                        break;
        
                    /*case LOADING_STATION:
                        auto.addCommands(new StartLeftToLoadingStation());
                        break;
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
