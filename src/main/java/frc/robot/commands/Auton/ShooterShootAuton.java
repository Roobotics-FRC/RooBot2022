package frc.robot.commands.Auton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterShootAuton extends CommandBase {
    private Shooter shooter;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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
        shooter.setVelocity(visionGetShooterSpeed());
        if (Math.abs(shooter.getVelocity() - visionGetShooterSpeed()) < 300) {
            shooter.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopFeeder();
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
        return 163457 - 22014*x + 2089*Math.pow(x, 2) - 56*Math.pow(x, 3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
