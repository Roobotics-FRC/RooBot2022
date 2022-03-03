package frc.robot.commands.Teleop;

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
        double y1 = OI.getInstance().getCyleController().getAxis(1);
        double y2 = OI.getInstance().getCyleController().getAxis(4);

        drivetrain.setLeftPercentOutput(y1);
        drivetrain.setRightPercentOutput(y2);
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