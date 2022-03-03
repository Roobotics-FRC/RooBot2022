// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.DriveDistanceAuton;
import frc.robot.commands.Auton.IntakeAuton;
import frc.robot.commands.Auton.IntakeDeployAuton;
import frc.robot.commands.Auton.ShooterShootAuton;
import frc.robot.commands.Teleop.DriveTurnToAngle;
import frc.robot.commands.Teleop.DriveWithJoystick;
import frc.robot.commands.Teleop.IntakeDefaultCommand;
import frc.robot.commands.Teleop.ShooterShootCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private PowerDistribution pdp;
  private PneumaticsControlModule pcm;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    pdp = new PowerDistribution();
    pcm = new PneumaticsControlModule(RobotMap.PCM_PORT);

    Drivetrain.getInstance();
    Shooter.getInstance();
    Camera.getInstance();
    Intake.getInstance();

    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeDefaultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new DriveWithJoystick());
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterShootCommand());
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
   m_autonomousCommand = new SequentialCommandGroup(
     new IntakeDeployAuton().withTimeout(3),
     new ParallelCommandGroup(
      new DriveDistanceAuton(75),
      new IntakeAuton().withTimeout(4)
     ),
     new DriveTurnToAngle().withTimeout(3),
     new ShooterShootAuton().withTimeout(8)
   );
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