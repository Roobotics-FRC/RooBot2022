package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngle extends CommandBase {
    public Drivetrain drivetrain;
    private double angle;
    private double timeout;
    
    public DriveTurnToAngle(double angle, double timeout) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
        this.angle = angle;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        drivetrain.enable();
        drivetrain.setSetpoint(angle);
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
        // return Math.abs(angle - drivetrain.getPigeonAngle()) < 5;
        return OI.getInstance().getDriveJoystick().getRawButtonPressed(5);
    }
}
