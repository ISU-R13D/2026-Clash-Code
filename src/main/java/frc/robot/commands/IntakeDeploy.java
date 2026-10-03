package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeploy extends Command {
    private final IntakeSubsystem intake;

    public IntakeDeploy(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!intake.isDeployed()) {
            intake.deploy();
        } else {
            intake.stopActuator();
            intake.setShakeSpeed(1);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.isDeployed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopActuator();
    }
}
