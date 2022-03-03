package frc.robot.commands.Teleop;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.disable();
    }

    @Override
    public boolean isFinished() {
        boolean killBtn = OI.getInstance().getOperatorController().getRawButton(RobotMap.KILL_COMMANDS_BUTTON);
        // boolean inTolerance = Math.abs((-table.getEntry("tx").getDouble(0)) - drivetrain.getPigeonAngle()) < 3;
        // return inTolerance || killBtn;
        return killBtn;
    }
}