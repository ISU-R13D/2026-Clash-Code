// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final SparkMax leftDriveFront;
  private final SparkMax leftDriveBack;
  private final SparkMax rightDriveFront;
  private final SparkMax rightDriveBack;

  private final SparkMaxConfig leftDriveFrontConfig;
  private final SparkMaxConfig leftDriveBackConfig;
  private final SparkMaxConfig rightDriveFrontConfig;
  private final SparkMaxConfig rightDriveBackConfig;

  private final DifferentialDrive drivetrain;

  private final DifferentialDrivePoseEstimator poseEstimator;
  private final DifferentialDriveKinematics kinematics;
  private final Field2d field;

  private final AHRS gyro = new AHRS(NavXComType.kMXP_UART);

  public DriveSubsystem() {
    leftDriveFront = new SparkMax(1, MotorType.kBrushless);
    leftDriveBack = new SparkMax(2, MotorType.kBrushless);
    rightDriveFront = new SparkMax(3, MotorType.kBrushless);
    rightDriveBack = new SparkMax(4, MotorType.kBrushless);


    leftDriveFrontConfig = new SparkMaxConfig();
    leftDriveBackConfig = new SparkMaxConfig();
    rightDriveFrontConfig = new SparkMaxConfig();
    rightDriveBackConfig = new SparkMaxConfig();

    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.kTrackWitdh));

    poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics, 
      getRotation2d(), 
      0, 
      0, 
      new Pose2d());

    field = new Field2d();


    configure();

    drivetrain = new DifferentialDrive(leftDriveBack::set, rightDriveBack::set);
  }



  private void configure() {
    leftDriveBackConfig
      .inverted(true)
      .openLoopRampRate(.5);

    leftDriveFrontConfig
      .inverted(true)
      .follow(leftDriveBack.getDeviceId())
      .openLoopRampRate(.5);

    rightDriveBackConfig
      .openLoopRampRate(.5);
    
    rightDriveFrontConfig
      .follow(rightDriveBack.getDeviceId())
      .openLoopRampRate(.5);

    leftDriveBack.configure(leftDriveBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftDriveFront.configure(leftDriveFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDriveBack.configure(rightDriveBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDriveFront.configure(rightDriveFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private Rotation2d getRotation2d() {
    return gyro.getRotation2d(); //TODO: see if we have gyro and make this work
  }

  private void updatePoseEstimation() {
    double leftEncoderValue = leftDriveBack.getEncoder().getPosition(); //Need  to convert to meters
    double rightEncoderValue = rightDriveBack.getEncoder().getPosition(); //Need to convert to meters

    poseEstimator.update(
      getRotation2d(), 
      leftEncoderValue,   
      rightEncoderValue);
  }

  public void drive(double joyLeft, double joyRight) {
    drivetrain.arcadeDrive(joyLeft, joyRight);
  }

  public void stop() {
    rightDriveBack.stopMotor();
    leftDriveBack.stopMotor();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    if (Constants.kEstimatePose) {
      updatePoseEstimation();

      field.setRobotPose(getPose());
      SmartDashboard.putData("Field", field);
    }
  }
}

