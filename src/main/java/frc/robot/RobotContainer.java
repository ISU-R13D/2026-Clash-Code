// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeRetract;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeSpin;
import frc.robot.subsystems.ShootSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final CommandXboxController m_driverOneController = new CommandXboxController(OperatorConstants.kDriverOneControllerPort);

  private final CommandXboxController m_driverTwoController = new CommandXboxController(OperatorConstants.kDriverTwoControllerPort);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem, m_driverOneController);

  private final ShootSubsystem m_shootSubsystem = new ShootSubsystem();

  private final ShootCommand m_shootCommand = new ShootCommand(m_shootSubsystem, m_driverOneController);

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  
  private final IntakeDeploy m_intakeDeploy = new IntakeDeploy(m_intakeSubsystem);
  
  private final IntakeRetract m_intakeRetract = new IntakeRetract(m_intakeSubsystem);

  private final IntakeSpin m_intakeSpin = new IntakeSpin(m_intakeSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverOneController.rightTrigger().whileTrue(m_shootCommand);
    m_driverOneController.rightBumper().onTrue(m_intakeDeploy);
    m_driverOneController.leftBumper().onTrue(m_intakeRetract);
    m_driverOneController.a().toggleOnTrue(m_intakeSpin);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public Command getTeleopCommand() {
    return m_driveCommand;
  }
}
