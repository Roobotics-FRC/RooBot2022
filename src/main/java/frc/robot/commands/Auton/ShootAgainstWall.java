package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShootAgainstWall extends CommandBase {
    private Shooter shooter;

    public ShootAgainstWall() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void initialize() {
        shooter.stop();
        shooter.setShooterFlat();
    }

    @Override
    public void execute() {
        shooter.setVelocity(RobotMap.SHOOTER_WALL_VELOCITY);
        if (Math.abs(shooter.getVelocity() - RobotMap.SHOOTER_WALL_VELOCITY) < 500) {
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
