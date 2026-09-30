package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double power;

    public IntakeCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.setShakeSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

