package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        startingRight = drivetrain.getRightPositionInches();
        startingLeft = drivetrain.getLeftPositionInches();
    }

    @Override
    public void execute() {
        drivetrain.setDistanceRight(distance + startingRight);
        drivetrain.setDistanceLeft(distance + startingLeft);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        boolean rightInRange = Math.abs(drivetrain.getRightPositionInches() - (distance + startingRight)) < 1;
        boolean leftInRange = Math.abs(drivetrain.getLeftPositionInches() - (distance + startingLeft)) < 1;
        return rightInRange & leftInRange;
    }
}