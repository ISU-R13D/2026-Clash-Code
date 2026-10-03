package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command {
    private final ShootSubsystem shooter;
    private final CommandXboxController controller;

    public ShootCommand(ShootSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.controller = null;
        addRequirements(shooter);

        this.fixedSpeed = speed;
    }

    public ShootCommand(ShootSubsystem shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.controller = controller;
        this.fixedSpeed = 0;
        addRequirements(shooter);
    }

    private final double fixedSpeed;

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (controller != null) {
            double speed = controller.getRightTriggerAxis();
            shooter.spinPercentage(.8);
        } else {
            shooter.spinRPM(fixedSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
