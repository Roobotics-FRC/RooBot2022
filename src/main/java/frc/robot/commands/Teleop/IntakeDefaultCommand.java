package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.RobotBase;
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
        intake.deployIntake();
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRawAxis(RobotMap.INTAKE_INTAKE_AXIS) > 0.05) {
            intake.setIntake(OI.getInstance().getOperatorController().getRawAxis(RobotMap.INTAKE_INTAKE_AXIS) * 0.75);
        } else if (OI.getInstance().getOperatorController().getRawAxis(RobotMap.INTAKE_REVERSE_INTAKE_AXIS) > 0.05) {
            intake.setIntake(-OI.getInstance().getOperatorController().getRawAxis(RobotMap.INTAKE_REVERSE_INTAKE_AXIS) * 0.75);
        } else {
            intake.stopIntake();
        }
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_FEED_BUTTON)) {
            intake.setIntake(0.25);
        }
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.INTAKE_DEPLOY_INTAKE_BUTTON)) {
            intake.deployIntake();
        } else if (OI.getInstance().getOperatorController().getRawButton(RobotMap.INTAKE_RETRACT_INTAKE_BUTTON)) {
            intake.retractIntake();
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
