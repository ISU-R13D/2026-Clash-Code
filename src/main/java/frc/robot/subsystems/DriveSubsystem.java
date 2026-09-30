// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.ResetMode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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

    kinematics = new DifferentialDriveKinematics(Constants.kTrackWitdh);

    poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics, 
      getRotation2d(), 
      0, 
      0, 
      new Pose2d());

    field = new Field2d();


    configure();
    configurePathplanner();

    drivetrain = new DifferentialDrive(leftDriveBack::set, rightDriveBack::set);
  }

  private void configurePathplanner() {
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();

          AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    //TODO: convert to m/s
    double leftVelocity = rpmToMetersPerSecond(leftDriveBack.getEncoder().getVelocity());
    double rightVelocity = rpmToMetersPerSecond(rightDriveBack.getEncoder().getVelocity());

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(wheelSpeeds);

    return speeds;
  }

  private double rpmToMetersPerSecond(double motorRPM) {
    double wheelRPM = motorRPM / Constants.kWheelGearRatio;
    double wheelCircumference = Math.PI * Constants.kWheelDiameter;

    return (wheelRPM / 60.0) * wheelCircumference;
  }

  private double metersPerSecondToRPM(double metersPerSecond) {
    double wheelCircumference = Math.PI * Constants.kWheelDiameter;
    double rotationsPerSecond = metersPerSecond / wheelCircumference;
    double wheelRPM = rotationsPerSecond * 60.0;

    return wheelRPM * Constants.kWheelGearRatio;
}

  private double rotationsToMeters(double motorRotations) {
    double wheelRotations = motorRotations / Constants.kWheelGearRatio;

    return wheelRotations * Math.PI * Constants.kWheelDiameter;
}

  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(speeds);

    leftDriveBack.getClosedLoopController().setSetpoint(
        metersPerSecondToRPM(wheelSpeeds.leftMetersPerSecond),
        ControlType.kVelocity
    );

    rightDriveBack.getClosedLoopController().setSetpoint(
        metersPerSecondToRPM(wheelSpeeds.rightMetersPerSecond),
        ControlType.kVelocity
    );
}

  private void configure() {
    leftDriveBackConfig
      .inverted(true)
      .openLoopRampRate(.5)
      .closedLoop.p(Constants.kDriveP);

    leftDriveFrontConfig
      .inverted(true)
      .follow(leftDriveBack.getDeviceId())
      .openLoopRampRate(.5);

    rightDriveBackConfig
      .openLoopRampRate(.5)
      .closedLoop.p(Constants.kDriveP);
    
    rightDriveFrontConfig
      .follow(rightDriveBack.getDeviceId())
      .openLoopRampRate(.5);

    leftDriveBack.configure(leftDriveBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftDriveFront.configure(leftDriveFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDriveBack.configure(rightDriveBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDriveFront.configure(rightDriveFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  private void updatePoseEstimation() {
    double leftEncoderValue = rotationsToMeters(leftDriveBack.getEncoder().getPosition());
    double rightEncoderValue = rotationsToMeters(rightDriveBack.getEncoder().getPosition());

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

