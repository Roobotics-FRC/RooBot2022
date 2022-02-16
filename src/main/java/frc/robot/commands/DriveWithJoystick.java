package frc.robot.commands;

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
        // double y = -OI.getInstance().getDriveJoystick().rooGetY();
        // double z = OI.getInstance().getDriveJoystick().rooGetZ();

        // boolean brake = OI.getInstance().getRightDriveJoystick().getRawButton(RobotMap.DRIVE_SLOWER_SPEED_BUTTON) || OI.getInstance().getLeftDriveJoystick().getRawButton(RobotMap.DRIVE_SLOWER_SPEED_BUTTON);
        
        // z = z * (-OI.getInstance().getDriveJoystick().rooGetThrottle() + 1) / 2;

        // 
        // drivetrain.setRight(y - z);
        // drivetrain.setLeft(y + z);

        double y1 = OI.getInstance().getCyleController().getAxis(1);
        double y2 = OI.getInstance().getCyleController().getAxis(4);

        // if (brake) {
        //     y1 = 0;
        //     y2 = 0;
        // }

        drivetrain.setLeft(y1);
        drivetrain.setRight(y2);
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