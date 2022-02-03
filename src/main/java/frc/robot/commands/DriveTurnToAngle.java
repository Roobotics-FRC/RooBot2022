package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngle extends PIDCommand {
    public Drivetrain drivetrain;
    
    // public DriveTurnToAngle(double targetAngle) {
    //     super(new PIDController(RobotMap.DRIVETRAIN_ANG_PID_GAINS.kP, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kI, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kD), arg1, arg2, arg3, arg4)
    //     addRequirements(this.drivetrain = Drivetrain.getInstance());
    // }
}
