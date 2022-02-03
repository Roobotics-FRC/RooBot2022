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
        double y = -OI.getInstance().getDriveJoystick().rooGetY();
        double z = OI.getInstance().getDriveJoystick().rooGetZ();

        boolean slowMode = OI.getInstance().getDriveJoystick().getRawButton(RobotMap.DRIVE_SLOWER_SPEED_BUTTON);
        
        z = z * (-OI.getInstance().getDriveJoystick().rooGetThrottle() + 1) / 2;

        if (slowMode) {
            y = y/4;
            z = z/4;
        }
        drivetrain.setRight(y - z);
        drivetrain.setLeft(y + z);
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