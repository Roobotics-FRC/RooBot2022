package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Intake;

public class IntakeDefaultCommand extends CommandBase {
    public Intake intake;

    public IntakeDefaultCommand() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void initialize() {
        intake.stopIntake();
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.INTAKE_INTAKE_BUTTON)) {
            intake.startIntake();
        } else {
            intake.stopIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
