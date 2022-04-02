package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeAuton extends CommandBase {
    public Intake intake;
    private double speed;

    public IntakeAuton(double speed) {
        addRequirements(this.intake = Intake.getInstance());
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        intake.deployIntake();
    }

    @Override
    public void execute() {
        intake.setIntake(speed);
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
