package frc.robot.commands;

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
        // if (OI.getInstance().getDriveJoystick().getRawButton(RobotMap.TEST_INTAKE)) {
        //     intake.startIntake();
        // } else {
        //     intake.stopIntake();
        // }
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
