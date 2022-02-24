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
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.CLIMB_RAISE_BUTTON)) {
            climb.raiseClimb();
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.CLIMB_LOWER_BUTTON)) {
            climb.lowerClimb();
        } else {
            climb.stop();
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