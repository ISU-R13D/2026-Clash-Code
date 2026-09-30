package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command {
    private final ShootSubsystem shooter;
    private final double speed;
    private final boolean speedIsRPM;

    public ShootCommand(ShootSubsystem shooter, double speed, boolean isRPM) {
        this.shooter = shooter;
        this.speed = speed;
        this.speedIsRPM = isRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(speedIsRPM)
            shooter.spinRPM(speed);
        else
            shooter.spinPercentage(speed);
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

