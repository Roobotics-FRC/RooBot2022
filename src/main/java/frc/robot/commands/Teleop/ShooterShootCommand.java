package frc.robot.commands.Teleop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Shooter;

public class ShooterShootCommand extends CommandBase {
    private Shooter shooter;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
        double speed = SmartDashboard.getNumber("SHOOTERSPEED", 0) * 100000;
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON)) {
            shooter.setVelocity(getShooterVelocityFromDistance(getDistanceFromYDist(table.getEntry("ty").getDouble(0))));
            if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_FEED_BUTTON)) {
                shooter.feed();
            } else {
                shooter.stopFeeder();
            }
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON)) {
            shooter.setVelocity(speed);
            if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_FEED_BUTTON)) {
                shooter.feed();
            } else {
                shooter.stopFeeder();
            }
        } else {
            shooter.stop();
            shooter.stopFeeder();
        }
        SmartDashboard.putNumber("ShooterVelocity", shooter.getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    private double getDistanceFromYDist(double yDist) {
        return yDist;
    }

    private double getShooterVelocityFromDistance(double distance) {
        return distance;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
