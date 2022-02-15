package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Shooter;

public class ShooterShootCommand extends CommandBase {
    public Shooter shooter;

    public ShooterShootCommand() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void initialize() {
        shooter.stop();
        SmartDashboard.putNumber("SHOOTERSPEED", 0);
    }

    @Override
    public void execute() {
        // double speed = (-OI.getInstance().getCyleController().getAxis(2) + 1) / 2;
        double speed = SmartDashboard.getNumber("SHOOTERSPEED", 0);
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON)) {
            shooter.setSpeed(speed);
            // shooter.feed();
        } else {
            shooter.setSpeed(0);
            shooter.stopFeeder();
        }
        SmartDashboard.putNumber("ShooterVelocity", shooter.getVelocity());
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
