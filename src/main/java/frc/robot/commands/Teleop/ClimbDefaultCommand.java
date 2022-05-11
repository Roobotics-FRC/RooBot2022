package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Climb;

public class ClimbDefaultCommand extends CommandBase {
    private Climb climb;

    public ClimbDefaultCommand() {
        addRequirements(this.climb = Climb.getInstance());
    }

    @Override
    public void initialize() {
        climb.stop();
        climb.retractArms();
    }

    @Override
    public void execute() {
        if (Math.abs(OI.getInstance().getOperatorController().getRawAxis(RobotMap.CLIMB_AXIS)) > 0.15) {
            climb.setClimb(OI.getInstance().getOperatorController().getAxis(RobotMap.CLIMB_AXIS) * 0.8);
        } else {
            climb.stop();
        }
        if (OI.getInstance().getCyleController().getRawButtonPressed(RobotMap.TOGGLE_CLIMB_ARMS)) {
            climb.retractArms();
        } else if (OI.getInstance().getCyleController().getRawButtonReleased(RobotMap.TOGGLE_CLIMB_ARMS)) {
            climb.deployArms();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}