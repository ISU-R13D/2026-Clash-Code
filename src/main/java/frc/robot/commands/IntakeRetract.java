package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends Command {
    private final IntakeSubsystem intake;

    public IntakeRetract(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!intake.isRetracted()) {
            intake.retract();
            intake.stopShake();
        } else {
            intake.stopActuator();
        }
    }

    @Override
    public boolean isFinished() {
        return intake.isRetracted();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopActuator();
    }
}
