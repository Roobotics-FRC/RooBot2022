package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeAuton extends CommandBase {
    public Intake intake;

    public IntakeAuton() {
        addRequirements(this.intake = Intake.getInstance());
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        intake.deployIntake();
    }

    @Override
    public void execute() {
        intake.setIntake(0.8);
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
