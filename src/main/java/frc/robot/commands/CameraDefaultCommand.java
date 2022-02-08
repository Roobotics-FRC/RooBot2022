package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
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
    public void execute() {
        if (OI.getInstance().getDriveJoystick().getRawButton(RobotMap.CAMERA_PROCESS_BUTTON)) {
            camera.processFrame();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
