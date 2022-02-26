package frc.robot.commands.Teleop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
        double speed = 0;
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON)) {
            speed = getShooterVelocityFromDistance(getDistanceFromYDist(table.getEntry("ty").getDouble(0)));
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - getShooterVelocityFromDistance(getDistanceFromYDist(table.getEntry("ty").getDouble(0)))) < 300) {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.5);
            } else {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON)) {
            speed = SmartDashboard.getNumber("SHOOTERSPEED", 0) * 35000;
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < 300) {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.5);
            } else {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else {
            speed = 0;
            OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            shooter.stop();
        }
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_FEED_BUTTON)) {
            shooter.feed();
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_REVERSE_FEED_BUTTON)) {
            shooter.reverseFeed();
        } else {
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
