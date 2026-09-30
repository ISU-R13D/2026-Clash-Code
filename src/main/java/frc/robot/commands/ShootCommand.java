package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command {
    private final ShootSubsystem shooter;
    private final double speed;
    private final boolean speedIsRPM;

    public ShootCommand(ShootSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        this.speedIsRPM = true;
        addRequirements(shooter);
    }

    public ShootCommand(ShootSubsystem shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.speed = controller.getRightTriggerAxis(); //Ideally we know where we are on the field and use that to set the speed
        this.speedIsRPM = false;
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

