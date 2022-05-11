package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * Holds various mappings and constants.
 */

public class RobotMap {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getDistanceFromCamera() {
        double x = table.getEntry("ty").getDouble(0);
        return 10.6 - 0.441 * x + 0.0262 * Math.pow(x, 2) - 0.000896 * Math.pow(x, 3);
    }

    public static double getShooterVelocityFromDistanceFar() {
        double x = getDistanceFromCamera();
        return -195714 + 65605 * x + -5149 * Math.pow(x, 2) + 139 * Math.pow(x, 3);
    }
    public static double getShooterVelocityFromDistanceClose() {
        double x = getDistanceFromCamera();
        return -1840000.0 + 948889 * x - 162769 * Math.pow(x, 2) + 9814 * Math.pow(x, 3) + 98.8 * Math.pow(x, 4) - 20.7 * Math.pow(x, 5);
    }

    // Button Ids
    // OPERATOR CONTROLLER
    public static final int SHOOTER_SHOOT_BUTTON = 2;
    public static final int SHOOTER_SHOOT_WITH_VISION_BUTTON = 1;
    public static final int SHOOTER_SHOOT_WITH_VISION_BUTTON_MANUAL = 3;
    public static final int SHOOTER_SHOOT_SIDE_WALL_BUTTON = 4;
    public static final int SHOOTER_FEED_BUTTON = 5;
    public static final int SHOOTER_REVERSE_FEED_BUTTON = 6;
    public static final int INTAKE_INTAKE_AXIS = 2;
    public static final int INTAKE_REVERSE_INTAKE_AXIS = 3;
    public static final int INTAKE_DEPLOY_INTAKE_BUTTON = 9;
    public static final int INTAKE_RETRACT_INTAKE_BUTTON = 10;
    public static final int CLIMB_AXIS = 1;
    public static final int DRIVE_TURN_TO_TARGET_BUTTON = 7;
    public static final int KILL_COMMANDS_BUTTON = 8;
    // CYLE CONTROLLER
    public static final int DRIVE_DISTANCE_BUTTON = 14;
    public static final int DRIVE_SLOWER_SPEED_BUTTON = 7;
    public static final int DRIVE_LEFT_AXIS = 1;
    public static final int DRIVE_RIGHT_AXIS = 4;
    public static final int DRIVER_INTAKE_DEPLOY_INTAKE_BUTTON = 2;
    public static final int DRIVER_INTAKE_RETRACT_INTAKE_BUTTON = 3;
    public static final int TOGGLE_CLIMB_ARMS = 12;

    // OI devices
    public static final int LEFT_DRIVE_JOYSTICK_PORT = 0;
    public static final int RIGHT_DRIVE_JOYSTICK_PORT = 1;
    public static final int CYCLE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double JOYSTICK_DEFAULT_DEADZONE = 0.09;
    public static final int PIGEON_ID = 19;
    public static final int COMPRESSOR_PORT = 2;
    public static final int PCM_PORT = 3;
    public static final int INTAKE_DEPLOY_SOLENOID_DEPLOY = 7;
    public static final int INTAKE_DEPLOY_SOLENOID_RETRACT = 0;
    public static final int SHOOTER_ANGLE_SOLENOID_DEPLOY = 6;
    public static final int SHOOTER_ANGLE_SOLENOID_RETRACT = 1;
    public static final int CLIMB_ARM_SOLENOID_DEPLOY = 5;
    public static final int CLIMB_ARM_SOLENOID_RETRACT = 2;
    public static final int BOTTOM_LIMIT_SWITCH_DIO_PORT = 1;
    public static final int TOP_LIMIT_SWITCH_DIO_PORT = 0;

    // Motor Configs
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_1 = new MotorConfig(10, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_2 = new MotorConfig(11, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_3 = new MotorConfig(12, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_1 = new MotorConfig(13, false, true);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_2 = new MotorConfig(14, false, true);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_3 = new MotorConfig(15, false, true);

    public static final MotorConfig SHOOTER_MOTOR_1 = new MotorConfig(21, true, true);
    public static final MotorConfig SHOOTER_MOTOR_2 = new MotorConfig(22, true, false);
    public static final MotorConfig SHOOTER_FEEDER_MOTOR = new MotorConfig(23, true, true);

    public static final MotorConfig CLIMB_MOTOR_1 = new MotorConfig(41, true, false);
    public static final MotorConfig CLIMB_MOTOR_2 = new MotorConfig(42, false, false);

    public static final MotorConfig INTAKE_MOTOR = new MotorConfig(31, false, true);

    // PID GAINS
    public static final PID DRIVETRAIN_ANG_PID_GAINS = new PID(0, 0.01, 0.03, 0);
    public static final PID DRIVETRAIN_TALON_PID_GAINS = new PID(0, 0.002, 0.000003, 0);

    public static final PID SHOOTER_PID_GAINS_FAR = new PID(0.0075, 0.3, 0.00001, 3);
    public static final int SHOOTER_FAR_ID = 0;

    public static final PID SHOOTER_PID_GAINS_MID = new PID(0.007, 0.15, 0.00002, 4);
    public static final int SHOOTER_MID_ID = 1;

    public static final PID SHOOTER_PID_GAINS_CLOSE = new PID(0, 0.03, 0.0001, 0);
    public static final int SHOOTER_CLOSE_ID = 2;

    // Constants
    public static final double VISION_ALIGN_OFFSET = 0.3;
    public static final double SHOOTER_SETPOINT_THRESHOLD = 1250;
    public static final double ALIGN_ANGLE_THRESHOLD = 3;
    public static final double ENCODER_COUNTS_PER_REV = 4096;
    public static final double DRIVE_COUNTS_PER_REV = 2048;
    public static final double DRIVE_GEAR_RATIO = 10.86;
    public static final double DRIVE_WHEEL_CIRCUMFRENCE = 6 * Math.PI;
    public static final DoubleSolenoid.Value DEPLOYED = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kReverse;

    // SHOOTER CONSTANTS
    public static final double SHOOTER_WALL_VELOCITY = 85000;
    public static final double SHOOTER_LOW_VELOCITY = 40000;
    public static final double SHOOTER_SIDE_WALL_VELOCITY = 91000;
    
    // Conversion factors
    // public static final double ENCODER_UNITS_TO_INCHES = DRIVE_WHEEL_DIAMETER_IN * Math.PI / DRIVE_ENCODER_COUNTS_PER_REV / DRIVE_GEARBOX_RATIO;

    // Utility classes
    public static final class MotorConfig {
        public final int id;
        public final boolean inverted;
        public final NeutralMode neutralMode;
        public final boolean encoderPhase;
        public final PID gains;

        /**
         * Constructs a new MotorConfig for a motor using closed-loop control.
         * @param id the CAN ID of the motor.
         * @param inverted whether to invert motor output values.
         * @param neutralMode the motor's neutral mode.
         * @param encoderPhase whether the motor is out of phase with its sensor.
         * @param gains the PID gains for this motor's closed-loop control.
         */
        public MotorConfig(int id, boolean inverted,
                           NeutralMode neutralMode, boolean encoderPhase, PID gains) {
            this.id = id;
            this.inverted = inverted;
            this.neutralMode = neutralMode;
            this.encoderPhase = encoderPhase;
            this.gains = gains;
        }

        /**
         * Constructs a new MotorConfig for a motor not under closed-loop control.
         * @param id the CAN ID of the motor.
         * @param inverted whether to invert motor output values.
         * @param encoderPhase whether the motor is out of phase with its sensor.
         */
        public MotorConfig(int id, boolean inverted, boolean encoderPhase) {
            this.id = id;
            this.inverted = inverted;
            this.encoderPhase = encoderPhase;

            this.neutralMode = NeutralMode.Brake;
            this.gains = new PID(0, 0, 0,0);
        }
        /**
         * Constructs a new MotorConfig for a motor not under closed-loop control.
         * @param id the CAN ID of the motor.
         * @param inverted whether to invert motor output values.
         * @param neutralMode the motor's neutral mode.
         */
        public MotorConfig(int id, boolean inverted, NeutralMode neutralMode) {
            this.id = id;
            this.inverted = inverted;
            this.neutralMode = neutralMode;

            this.encoderPhase = false;
            this.gains = new PID(0, 0, 0,0);
        }
    }

    // PID- and sensor-related constants
    public static final class PID {
        public final double kF;
        public final double kP;
        public final double kI;
        public final double kD;

        /**
         * Constructs a new PID parameters object.
         * @param kF feed-forward gain.
         * @param kP proportional gain.
         * @param kI integral gain.
         * @param kD derivative gain.
         */
        public PID(double kF, double kP, double kI, double kD) {
            this.kF = kF;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public static double constrainPercentOutput(double input) {
        return Math.max(Math.min(input, 1), -1);
    }
    public static double pidConstrainPercentOutput(double input) {
        return Math.max(Math.min(input, 0.5), -0.5);
    }
}