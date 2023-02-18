package frc.robot.commands.Auton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class DriveInCommand extends CommandBase {
    private Drivetrain drivetrain;
    private RamseteCommand command;

    public DriveInCommand() {
        addRequirements(drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        DifferentialDriveVoltageConstraint vConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotMap.FEED_FORWARD_kS, RobotMap.FEED_FORWARD_kV, RobotMap.FEED_FORWARD_kA), RobotMap.DRIVE_KINEMATICS, 6);

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(RobotMap.MAX_VELOCITY, RobotMap.MAX_ACCELERATION).setKinematics(RobotMap.DRIVE_KINEMATICS).addConstraint(vConstraint);
        trajectoryConfig.setReversed(false);
        
        Trajectory trajectory = RobotMap.DRIVE_IN_TRAJECTORY;

        command = new RamseteCommand(
            trajectory,
            drivetrain::getPose,
            new RamseteController(RobotMap.RAMSETE_B, RobotMap.RAMSETE_ZETA),
            new SimpleMotorFeedforward(RobotMap.FEED_FORWARD_kS, RobotMap.FEED_FORWARD_kV, RobotMap.FEED_FORWARD_kA),
            RobotMap.DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(RobotMap.FEED_BACK_VEL_kP, 0, RobotMap.FEED_BACK_VEL_kD),
            new PIDController(RobotMap.FEED_BACK_VEL_kP, 0, RobotMap.FEED_BACK_VEL_kD),
            drivetrain::setMotorVoltage,
            drivetrain);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void schedule() {
        command.schedule();
    }
}
