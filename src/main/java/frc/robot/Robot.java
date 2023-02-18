// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.DriveInCommand;
import frc.robot.commands.Auton.DriveOutCommand;
import frc.robot.commands.Teleop.DriveWithJoystick;
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

  String outTrajectoryPath = "paths/out_path.wpilib.json";
  String inTrajectoryPath = "paths/in_path.wpilib.json";

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

    // Set out_path trajectory
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(outTrajectoryPath);
      RobotMap.DRIVE_OUT_TRAJECTORY = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + outTrajectoryPath, ex.getStackTrace());
    }

   // Set in_path trajectory
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(inTrajectoryPath);
      RobotMap.DRIVE_IN_TRAJECTORY = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + inTrajectoryPath, ex.getStackTrace());
    }
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

    SmartDashboard.putNumber("CHASSIS_pX", Drivetrain.getInstance().getPose().getX());
    SmartDashboard.putNumber("CHASSIS_pY", Drivetrain.getInstance().getPose().getY());

    SmartDashboard.putNumber("Left_Vel", Drivetrain.getInstance().getLeftVelocity());
    SmartDashboard.putNumber("Right_Vel", Drivetrain.getInstance().getRightVelocity());
    SmartDashboard.putNumber("Chassis_vX", Drivetrain.getInstance().getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis_vY", Drivetrain.getInstance().getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis_vO",  Units.radiansToDegrees(Drivetrain.getInstance().getChassisSpeeds().omegaRadiansPerSecond));

    SmartDashboard.putNumber("Pidgeon_Degrees", Drivetrain.getInstance().getPigeonAngle());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new SequentialCommandGroup(new DriveOutCommand(), new DriveInCommand());

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