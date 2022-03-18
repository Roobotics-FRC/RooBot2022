package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Shooter;

public class ShooterShootCommand extends CommandBase {
    private Shooter shooter;

    public ShooterShootCommand() {
        addRequirements(this.shooter = Shooter.getInstance());
    }

    @Override
    public void initialize() {
        shooter.stop();
        shooter.setShooterAngled();
    }

    @Override
    public void execute() {
        double speed = 0;
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON)) {
            shooter.setShooterAngled();
            speed = RobotMap.visionGetShooterSpeed();
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < RobotMap.SHOOTER_SETPOINT_THRESHOLD) {
                SmartDashboard.putBoolean("SHOOTER_READY", true);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.8);
            } else {
                SmartDashboard.putBoolean("SHOOTER_READY", false);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON)) {
            shooter.setShooterAngled();
            speed = RobotMap.SHOOTER_LOW_VELOCITY;
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < RobotMap.SHOOTER_SETPOINT_THRESHOLD) {
                SmartDashboard.putBoolean("SHOOTER_READY", true);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.8);
            } else {
                SmartDashboard.putBoolean("SHOOTER_READY", false);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_SIDE_WALL_BUTTON)) {
            shooter.setShooterAngled();
            speed = RobotMap.SHOOTER_SIDE_WALL_VELOCITY;
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < RobotMap.SHOOTER_SETPOINT_THRESHOLD) {
                SmartDashboard.putBoolean("SHOOTER_READY", true);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.8);
            } else {
                SmartDashboard.putBoolean("SHOOTER_READY", false);
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else {
            SmartDashboard.putBoolean("SHOOTER_READY", false);
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
        SmartDashboard.putNumber("DISTANCE", RobotMap.getDistanceFromCamera());
        SmartDashboard.putNumber("SPEED", RobotMap.getShooterVelocityFromDistance());
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
