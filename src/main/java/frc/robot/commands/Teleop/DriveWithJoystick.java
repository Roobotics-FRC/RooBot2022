package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        // CYLE CONTROL EXPERIMENTAL
        // double y = OI.getInstance().getCyleController().getAxis(4);
        // double z = OI.getInstance().getCyleController().getAxis(0);

        // drivetrain.setLeftPercentOutput(y + z);
        // drivetrain.setRightPercentOutput(y - z);


        // CYLE CONTROL MARK I
        double y1 = filterCyleController(OI.getInstance().getCyleController().getAxis(1) * 2);
        double y2 = filterCyleController(OI.getInstance().getCyleController().getAxis(4) * 2);

        drivetrain.setLeftPercentOutput(y1);
        drivetrain.setRightPercentOutput(y2);
    }

    private double filterCyleController(double value) {
        if (Math.abs(value) < 0.15) {
            return 0;
        } else {
            return value;
        }
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