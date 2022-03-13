package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShooterShootAuton extends CommandBase {
    private Shooter shooter;
    

    public ShooterShootAuton() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void initialize() {
        shooter.stop();
        shooter.setShooterAngled();
    }

    @Override
    public void execute() {
        shooter.setVelocity(RobotMap.visionGetShooterSpeed(), false);
        if (Math.abs(shooter.getVelocity() - RobotMap.visionGetShooterSpeed()) < 500) {
            shooter.feed();
        } else {
            shooter.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
