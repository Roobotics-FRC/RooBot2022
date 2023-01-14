// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.DriveDistanceAuton;
import frc.robot.commands.Teleop.DriveWithJoystick;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PowerDistribution pdp;
  private Compressor compressor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    pdp = new PowerDistribution();
    compressor = new Compressor(RobotMap.PCM_PORT, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    Drivetrain.getInstance();
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new DriveWithJoystick());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("BatteryValue", pdp.getVoltage());
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    DifferentialDriveVoltageConstraint vConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotMap.FEED_FORWARD_kS, RobotMap.FEED_FORWARD_kV, RobotMap.FEED_FORWARD_kA), RobotMap.DRIVE_KINEMATICS, 10);

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(RobotMap.MAX_VELOCITY, RobotMap.MAX_ACCELERATION).setKinematics(RobotMap.DRIVE_KINEMATICS).addConstraint(vConstraint);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, 0)),
      new Pose2d(3, 0, new Rotation2d(0)),
      trajectoryConfig);

    Drivetrain.getInstance().resetOdometry(trajectory.getInitialPose());

    Drivetrain drivetrain = Drivetrain.getInstance();
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(RobotMap.RAMSETE_B, RobotMap.RAMSETE_ZETA),
        new SimpleMotorFeedforward(RobotMap.FEED_FORWARD_kS, RobotMap.FEED_FORWARD_kV, RobotMap.FEED_FORWARD_kA),
        RobotMap.DRIVE_KINEMATICS,
        drivetrain::getWheelSpeeds,
        new PIDController(RobotMap.FEED_BACK_VEL_kP, 0, 0),
        new PIDController(RobotMap.FEED_BACK_VEL_kP, 0, 0),
        drivetrain::setMotorVoltage,
        drivetrain);

      m_autonomousCommand = ramseteCommand.andThen(() -> drivetrain.setMotorVoltage(0, 0));

      if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
      }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
   if (m_autonomousCommand != null) {
     m_autonomousCommand.cancel();
   }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
  //  CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}