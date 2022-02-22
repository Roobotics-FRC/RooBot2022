package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;

public class CameraDefaultCommand extends CommandBase {
    public Camera camera;

    public CameraDefaultCommand() {
        addRequirements(this.camera = Camera.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
