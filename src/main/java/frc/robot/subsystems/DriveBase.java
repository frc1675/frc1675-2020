/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Field2d;

public class DriveBase extends SubsystemBase {
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;
  private WPI_TalonSRX leftFrontSim;
  private WPI_TalonSRX leftBackSim;
  private WPI_TalonSRX rightFrontSim;
  private WPI_TalonSRX rightBackSim;
  private CANEncoder leftAlternateEncoder;
  private CANEncoder rightAlternateEncoder;
  private double leftSimDistance;
  private double rightSimDistance;
  private AHRS navx;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;
  private ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drive Base");

  // Simulation variables
  private Field2d field2d;
  private DifferentialDriveWheelSpeeds simWheelSpeeds;
  private DifferentialDriveKinematics simKinematics;
  private DifferentialDriveOdometry odometry;
  private Double simLastOdometryUpdateTime = null;
  private double simLeftMeters = 0;
  private double simRightMeters = 0;
  private double simHeading = 0;  
  private double simDirection = 0;
  private double simStartX = 0;
  private double simStartY = 0;
  private Pose2d simPose;

  /**
   * Creates a new Drive.
   */
  public DriveBase() {
    if(RobotBase.isReal()){
      rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
      rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
      leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
      leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);

      leftAlternateEncoder = leftBack.getAlternateEncoder(kAltEncType, kCPR);
      rightAlternateEncoder = rightBack.getAlternateEncoder(kAltEncType, kCPR);

      //rightAlternateEncoder.setInverted(true);
      leftAlternateEncoder.setPosition(0);
      rightAlternateEncoder.setPosition(0);

      driveBaseTab.addNumber("Right position", () -> -rightAlternateEncoder.getPosition());
      driveBaseTab.addNumber("Left Position", () -> -leftAlternateEncoder.getPosition());

      driveBaseTab.addNumber("Right Front Output Current", () -> rightFront.getOutputCurrent());
      driveBaseTab.addNumber("Left Front Output Current", () -> leftFront.getOutputCurrent());
      driveBaseTab.addNumber("Right Back Output Current", () -> rightBack.getOutputCurrent());
      driveBaseTab.addNumber("Left Back Output Current", () -> leftBack.getOutputCurrent());
    }
    else {
      rightBackSim = new WPI_TalonSRX(Constants.RIGHT_BACK);
      rightFrontSim = new WPI_TalonSRX(Constants.RIGHT_FRONT);
      leftBackSim = new WPI_TalonSRX(Constants.LEFT_BACK);
      leftFrontSim = new WPI_TalonSRX(Constants.LEFT_FRONT);

      leftSimDistance = 0;
      rightSimDistance = 0;

      driveBaseTab.addNumber("Right position", () -> (rightSimDistance / Constants.ROTATIONS_PER_INCH));
      driveBaseTab.addNumber("Left Position", () -> (leftSimDistance / Constants.ROTATIONS_PER_INCH));

      field2d = new Field2d();
      simWheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
      simKinematics = new DifferentialDriveKinematics(0.5);
      odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    }
    
    navx = new AHRS(SerialPort.Port.kMXP);
    
    driveBaseTab.addNumber("Angle", () -> getAngle());
    driveBaseTab.addNumber("Heading", () -> getHeading());
    driveBaseTab.addNumber("Position", () -> getPosition());
    
  }

  public void setRightMotors(double power) {
    if(RobotBase.isReal()) {
      rightFront.set(-power);
      rightBack.set(-power);
    }
    else {
      if(power > 1) {
        power = 1;
      }
      else if(power < -1) {
        power = -1;
      }
      rightFrontSim.set(-power);
      rightBackSim.set(-power);
      
      rightSimDistance += power * Constants.ROTAIONS_PER_TICK;

      simWheelSpeeds.rightMetersPerSecond = Constants.SIM_ROBOT_METERS_PER_SECOND * power;
    }
  }

  public void setLeftMotors(double power) {
    if(RobotBase.isReal()) {
      leftFront.set(-power);
      leftBack.set(-power);
    }
    else {
      if(power > 1) {
        power = 1;
      }
      else if(power < -1) {
        power = -1;
      }
      leftFrontSim.set(-power);
      leftBackSim.set(-power);

      leftSimDistance += power * Constants.ROTAIONS_PER_TICK;

      simWheelSpeeds.leftMetersPerSecond = Constants.SIM_ROBOT_METERS_PER_SECOND * power;
    }
  }

  public double getPosition() {
    double averagePosition = 0;
    double leftEncoderValue = 0;
    double rightEncoderValue = 0;
    if(RobotBase.isReal()) {
      leftEncoderValue = -leftAlternateEncoder.getPosition();
      rightEncoderValue = -rightAlternateEncoder.getPosition();
      // SmartDashboard.putNumber("LeftEncoder", leftEncoderValue);
      // SmartDashboard.putNumber("RightEncoder", rightEncoderValue);
    }
    else {
      leftEncoderValue = leftSimDistance;
      rightEncoderValue = rightSimDistance;
    }
    averagePosition = (rightEncoderValue + leftEncoderValue) / 2;
    return averagePosition;
  }

  public void resetPosition() {
    if(RobotBase.isReal()) {
      leftAlternateEncoder.setPosition(0);
      rightAlternateEncoder.setPosition(0);
    }
    else {
      leftSimDistance = 0;
      rightSimDistance = 0;
    }
  }
  
  public void resetAngle() {
    if(RobotBase.isReal()) {
      navx.reset();
    }
    else {
      simHeading = 0;
    }
  }

  public void setStartPosition(Pose2d pose) {
    simStartX = pose.getTranslation().getX();
    simStartY = pose.getTranslation().getY();
    simDirection = -pose.getRotation().getDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      if (simLastOdometryUpdateTime == null) {
        simLastOdometryUpdateTime = Timer.getFPGATimestamp();
        return;
      }

      double dt = Timer.getFPGATimestamp() - simLastOdometryUpdateTime;
      simLeftMeters += simWheelSpeeds.leftMetersPerSecond * dt;
      simRightMeters += simWheelSpeeds.rightMetersPerSecond * dt;
      simHeading -= Math.toDegrees(simKinematics.toChassisSpeeds(simWheelSpeeds).omegaRadiansPerSecond) * 0.25 * dt;
      simDirection += Math.toDegrees(simKinematics.toChassisSpeeds(simWheelSpeeds).omegaRadiansPerSecond) * 0.25 * dt;
      if (simDirection > 180) {
        simDirection -= 360;
      } else if (simDirection < -180) {
        simDirection += 360;
      }
      odometry.update(Rotation2d.fromDegrees(simDirection), simLeftMeters, simRightMeters);

      double odometryX = odometry.getPoseMeters().getTranslation().getX();
      double odometryY = odometry.getPoseMeters().getTranslation().getY();

      simPose = new Pose2d(odometryX + simStartX, odometryY + simStartY, Rotation2d.fromDegrees(simDirection));

      /* Boundary protection */
      double simX = simPose.getTranslation().getX();
      double simY = simPose.getTranslation().getY();
      if(simX > Constants.FIELD_WIDTH) {
        odometry.resetPosition(new Pose2d(Constants.FIELD_WIDTH - simStartX, odometryY, odometry.getPoseMeters().getRotation()), Rotation2d.fromDegrees(simDirection));
        simLeftMeters = 0;
        simRightMeters = 0;
      }
      else if(simX < 0) {
        odometry.resetPosition(new Pose2d(0 - simStartX, odometryY, odometry.getPoseMeters().getRotation()), Rotation2d.fromDegrees(simDirection));
        simLeftMeters = 0;
        simRightMeters = 0;
      }
      if(simY > Constants.FIELD_HEIGHT) {
        odometry.resetPosition(new Pose2d(odometryX, Constants.FIELD_HEIGHT - simStartY, odometry.getPoseMeters().getRotation()), Rotation2d.fromDegrees(simDirection));
        simLeftMeters = 0;
        simRightMeters = 0;
      }
      else if(simY < 0) {
        odometry.resetPosition(new Pose2d(odometryX, 0 - simStartY, odometry.getPoseMeters().getRotation()), Rotation2d.fromDegrees(simDirection));
        simLeftMeters = 0;
        simRightMeters = 0;
      }

      simPose = new Pose2d(odometryX + simStartX, odometryY + simStartY, Rotation2d.fromDegrees(simDirection));

      /* odometry.resetPosition(beforeOdometryUpdate, beforeOdometryUpdate.getRotation());
        return; */

      // Still in boundary, update field2d
      
      field2d.setRobotPose(simPose);
      simLastOdometryUpdateTime = Timer.getFPGATimestamp();
    }
  }

  public double getAngle() {
    if(RobotBase.isReal()) {
      return navx.getAngle();
    }
    else {
      return simHeading;
    }
  }

  public double getHeading() {
    if(RobotBase.isReal()) {
      double angle = getAngle();
      double heading = (angle % 360);
      return heading;
    }
    else {
      return simHeading;
    }
  }

}
