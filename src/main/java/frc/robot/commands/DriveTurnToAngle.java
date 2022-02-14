package frc.robot.commands;

import org.opencv.features2d.AgastFeatureDetector;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngle extends CommandBase {
    public Drivetrain drivetrain;
    private double angle;
    private double timeout;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public DriveTurnToAngle(double angle, double timeout, boolean usingVision) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
        this.timeout = timeout;
        if (usingVision) {
            this.angle = table.getEntry("tx").getDouble(0);
            SmartDashboard.putNumber("angle", angle);
        } else {
            this.angle = angle;
        }
    }

    @Override
    public void initialize() {
        drivetrain.resetPigeonYaw();
        drivetrain.enable();
        // drivetrain.setSetpoint(90);
        drivetrain.setSetpoint(-table.getEntry("tx").getDouble(0));

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.disable();
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(angle - drivetrain.getPigeonAngle()) < 2;
        return false;
        // return OI.getInstance().getLeftDriveJoystick().getRawButtonPressed(RobotMap.DRIVE_TURN_90_BUTTON);
    }
}
