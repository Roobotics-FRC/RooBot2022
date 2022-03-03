package frc.robot.commands.Teleop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
        shooter.setShooterAngled();
    }

    @Override
    public void execute() {
        double speed = 0;
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON)) {
            shooter.setShooterAngled();
            speed = visionGetShooterSpeed();
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < 400) {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0.5);
            } else {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kRightRumble, 0);
            }
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON)) {
            shooter.setShooterFlat();
            speed = RobotMap.SHOOTER_WALL_VELOCITY;
            shooter.setVelocity(speed);
            if (Math.abs(shooter.getVelocity() - speed) < 400) {
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
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SET_ANGLE_ANGLED_BUTTON)) {
            shooter.setShooterAngled();
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SET_ANGLE_FLAT_BUTTON)) {
            shooter.setShooterFlat();
        }
        SmartDashboard.putNumber("ShooterVelocity", shooter.getVelocity());
        SmartDashboard.putNumber("DISTANCE", getDistanceFromCamera());
        SmartDashboard.putNumber("SPEED", getShooterVelocityFromDistance());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    private double visionGetShooterSpeed() {
        return getShooterVelocityFromDistance();
    }

    private double getDistanceFromCamera() {
        double x = table.getEntry("ty").getDouble(0);
        return 13 - 0.542*x + 0.0214*Math.pow(x, 2) - 0.00118*Math.pow(x, 3) + 0.0000267*Math.pow(x, 4);
    }

    private double getShooterVelocityFromDistance() {
        double x = getDistanceFromCamera();
        return 156714 - 20495 * x + 1977*Math.pow(x, 2) - 53.3*Math.pow(x, 3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
