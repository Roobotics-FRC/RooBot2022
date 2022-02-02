// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.RobotMap;
// import frc.robot.input.OI;
// import frc.robot.subsystems.Drivetrain;

// public class DriveStraightCommand extends PIDCommand {

//     // COPPIED FROM THE 2019 DRIVE STRAIGHT COMMAND

//     private static final long COOLDOWN = 500;

//     private long lastManualOp = 0;

//     private Drivetrain drivetrain;

//     /**
//      * Constructs a new teleop drive command that drives straight with PID.
//      */
//     public DriveStraightCommand() {
//         super("DriveStraightCommand", RobotMap.DRIVETRAIN_ANG_PID_GAINS.kP,
//                 RobotMap.DRIVETRAIN_ANG_PID_GAINS.kI, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kD);
//         addRequirements(this.drivetrain = Drivetrain.getInstance());
//     }

    

//     @Override
//     public void initialize() {
//         // Angular PID configuration
//         this.setSetpoint(drivetrain.getPigeonYaw());
//         this.getController().setOutputRange(-RobotMap.AUTON_TURN_SPEED,
//                 RobotMap.AUTON_TURN_SPEED);
//         this.getPIDController().setPID(RobotMap.DRIVETRAIN_ANG_PID_GAINS.kP,
//                 RobotMap.DRIVETRAIN_ANG_PID_GAINS.kI, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kD);
//         // this.getPIDController().setPID(SmartDashboard.getNumber("kP-T", 0),
//         //         SmartDashboard.getNumber("kI-T", 0),
//         //         SmartDashboard.getNumber("kD-T", 0));
//     }

//     @Override
//     public double returnPIDOutput() {
//         return this.drivetrain.getPigeonYaw();
//     }

//     @Override
//     public void usePIDOutput(double angleOutput) {
//         double joyZ = OI.getInstance().getDriveJoystick().rooGetZ();
//         double joyX = OI.getInstance().getDriveJoystick().rooGetX();
//         double joyY = OI.getInstance().getDriveJoystick().rooGetY();

//         double rightOutput = RobotMap.constrainPercentOutput(joyY + joyZ);
//         double leftOutput = RobotMap.constrainPercentOutput(joyY - joyZ);

//         if (joyZ != 0) this.lastManualOp = System.currentTimeMillis();

//         this.setSetpoint(drivetrain.getPigeonYaw());
//         this.drivetrain.setPercentOutput(Drivetrain.TalonID.RIGHT_1, rightOutput);
//         this.drivetrain.setPercentOutput(Drivetrain.TalonID.LEFT_1, leftOutput);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }