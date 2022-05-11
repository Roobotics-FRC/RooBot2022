package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Shooter;

public class ShooterFeedBackwardsAuton extends CommandBase {
    private Shooter shooter;
    

    public ShooterFeedBackwardsAuton() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.reverseFeedSlow();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
