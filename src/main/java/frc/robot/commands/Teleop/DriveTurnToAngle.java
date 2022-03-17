package frc.robot.commands.Teleop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngle extends CommandBase {
    public Drivetrain drivetrain;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public DriveTurnToAngle() {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        drivetrain.resetPigeonYaw();
        drivetrain.enable();
        drivetrain.setSetpoint(-table.getEntry("tx").getDouble(0));
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperatorController().getRawButton(RobotMap.DRIVE_TURN_TO_TARGET_BUTTON)) {
            drivetrain.resetPigeonYaw();
            drivetrain.enable();
            drivetrain.setSetpoint(-table.getEntry("tx").getDouble(0));
        }
        if (Math.abs(table.getEntry("tx").getDouble(0)) < RobotMap.ALIGN_ANGLE_THRESHOLD) {
            if (!OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON) && !OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON)) {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0.2);
            } else {
                OI.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0);
            }
        } else {
            OI.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.disable();
        OI.getInstance().getOperatorController().setRumble(RumbleType.kLeftRumble, 0);
    }

    @Override
    public boolean isFinished() {
        boolean killBtn = OI.getInstance().getOperatorController().getRawButton(RobotMap.KILL_COMMANDS_BUTTON);
        boolean leftKill = Math.abs(OI.getInstance().getCyleController().getAxis(RobotMap.DRIVE_LEFT_AXIS)) > 0.12;
        boolean rightKill = Math.abs(OI.getInstance().getCyleController().getAxis(RobotMap.DRIVE_RIGHT_AXIS)) > 0.12;
        // boolean shooting = OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_WITH_VISION_BUTTON) || OI.getInstance().getOperatorController().getRawButton(RobotMap.SHOOTER_SHOOT_BUTTON);
        // boolean inTolerance = Math.abs((-table.getEntry("tx").getDouble(0)) - drivetrain.getPigeonAngle()) < 3;
        // return inTolerance || killBtn;
        return killBtn || leftKill || rightKill;
    }
}