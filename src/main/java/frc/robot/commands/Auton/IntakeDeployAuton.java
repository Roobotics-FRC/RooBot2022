package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDeployAuton extends CommandBase {
    public Intake intake;

    public IntakeDeployAuton() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void initialize() {
        intake.deployIntake();
    }

    @Override
    public void execute() {
        intake.deployIntake();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
