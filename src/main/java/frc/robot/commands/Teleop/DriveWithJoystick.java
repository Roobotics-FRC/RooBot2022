package frc.robot.commands.Teleop;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends CommandBase {
    public Drivetrain drivetrain;

    public DriveWithJoystick() {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        drivetrain.stop();
    }

    @Override
    public void execute() {
        drivetrain.drive(OI.getInstance().getCyleController().getRawAxis(RobotMap.DRIVE_LEFT_AXIS), OI.getInstance().getCyleController().getRawAxis(RobotMap.DRIVE_RIGHT_AXIS));

        drivetrain.updateOdometry();

        SmartDashboard.putNumber("CHASSIS_pX", drivetrain.getPose().getX());
        SmartDashboard.putNumber("CHASSIS_pY", drivetrain.getPose().getY());

        SmartDashboard.putNumber("Left_Vel", drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Right_Vel", drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Chassis_vX", drivetrain.getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis_vY", drivetrain.getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis_vO",  Units.radiansToDegrees(drivetrain.getChassisSpeeds().omegaRadiansPerSecond));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}