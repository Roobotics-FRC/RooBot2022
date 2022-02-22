package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDeployCommand extends CommandBase {
    public Intake intake;

    public IntakeDeployCommand() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void initialize() {
        intake.stopIntake();
    }

    @Override
    public void execute() {
        intake.deployIntake();
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
