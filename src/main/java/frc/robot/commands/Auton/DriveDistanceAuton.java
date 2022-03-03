package frc.robot.commands.Auton;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceAuton extends CommandBase {
    private Drivetrain drivetrain;
    private double distance;
    private double startingRight;
    private double startingLeft;


    public DriveDistanceAuton(double distance) {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
        this.distance = distance;
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        drivetrain.setNeutralMode(NeutralMode.Coast);
        startingRight = drivetrain.getRightPositionInches();
        startingLeft = drivetrain.getLeftPositionInches();
        drivetrain.setDistanceRight(distance + startingRight);
        drivetrain.setDistanceLeft(distance + startingLeft);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public boolean isFinished() {
        boolean rightInRange = Math.abs(drivetrain.getRightPositionInches() - (distance + startingRight)) < 2;
        boolean leftInRange = Math.abs(drivetrain.getLeftPositionInches() - (distance + startingLeft)) < 2;
        return rightInRange && leftInRange;
        // return OI.getInstance().getCyleController().getRawButton(RobotMap.KILL_COMMANDS_BUTTON);
        
    }
}