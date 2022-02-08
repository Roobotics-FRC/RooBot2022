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
    }

    @Override
    public void execute() {
        double speed = (-OI.getInstance().getDriveJoystick().getThrottle() + 1) / 2;
        if (OI.getInstance().getDriveJoystick().getRawButton(RobotMap.TEST_SHOOTER)) {
            shooter.setSpeed(-speed);
        } else {
            shooter.setSpeed(0);
        }
        SmartDashboard.putNumber("ShooterSpeed", speed);
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
