package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngle extends CommandBase {
    public Drivetrain drivetrain;
    private boolean finished = false;
    private double angle;
    
    public DriveTurnToAngle(double angle) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
        this.angle = angle;
    }

    @Override
    public void initialize() {
        drivetrain.enable();
        drivetrain.setSetpoint(angle);
    }

    @Override
    public void execute() {
        finished = drivetrain.getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        drivetrain.disable();
        return finished;
    }
}
