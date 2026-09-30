// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class ShootSubsystem extends SubsystemBase {

  private final SparkMax shooterLeader = new SparkMax(OperatorConstants.shooterLeaderID, MotorType.kBrushless);
  private final SparkMax shooterFollower = new SparkMax(OperatorConstants.shooterFollowerID, MotorType.kBrushless);

  //For RPM control
  private final SparkClosedLoopController shooterController = shooterLeader.getClosedLoopController();

  //Encode for Drivers Station Panel speed value
  private final SparkAbsoluteEncoder shooterEncoder = shooterLeader.getAbsoluteEncoder(); 
  
  public ShootSubsystem() {
    SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();

    shooterLeaderConfig
    .inverted(false);

    shooterLeaderConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.0001, 0.0, 0.0);

    SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();

    shooterFollowerConfig
    .follow(OperatorConstants.shooterLeaderID)
    .inverted(true);

    shooterLeader.configure(shooterLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configure(shooterFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);



  }

  public Command spinPercentage(double speed) {
    if(speed == 0.0)
      return this.runOnce(() -> shooterLeader.stopMotor());

    return this.startEnd(() -> shooterLeader.set(speed), () -> shooterLeader.stopMotor());
  }

  public Command spinRPM(double rpm) {
    return this.startEnd(() -> shooterController.setSetpoint(rpm, ControlType.kVelocity), () -> shooterLeader.stopMotor());
  }


  public Command stop(){
    return this.runOnce(() -> shooterLeader.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
