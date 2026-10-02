package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double actuatorSpeed;
    private final double shakeSpeed;

    public IntakeCommand(IntakeSubsystem intake, double actuatorSpeed, double shakeSpeed) {
        this.intake = intake;
        this.actuatorSpeed = actuatorSpeed;
        this.shakeSpeed = shakeSpeed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setActuatorSpeed(actuatorSpeed);
        intake.setShakeSpeed(shakeSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

