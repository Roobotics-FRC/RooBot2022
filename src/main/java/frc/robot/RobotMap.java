package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;


/**
 * Holds various mappings and constants.
 */

public class RobotMap {

    // Button Ids
    public static final int TEST_SHOOTER = 11;
    public static final int CAMERA_PROCESS_BUTTON = 12;
    public static final int DRIVE_TURN_90_BUTTON = 6;
    public static final int DRIVE_SLOWER_SPEED_BUTTON = 2;

    // OI devices
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;
    public static final double JOYSTICK_DEFAULT_DEADZONE = 0.09;
    public static final int PIGEON_ID = 19;

    // Motor Configs
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_1 = new MotorConfig(10, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_2 = new MotorConfig(11, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_RIGHT_3 = new MotorConfig(12, true, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_1 = new MotorConfig(13, false, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_2 = new MotorConfig(14, false, false);
    public static final MotorConfig DRIVETRAIN_MOTOR_LEFT_3 = new MotorConfig(15, false, false);

    public static final MotorConfig SHOOTER_MOTOR_1 = new MotorConfig(21, false, true);
    public static final MotorConfig SHOOTER_MOTOR_2 = new MotorConfig(22, false, false);

    public static final PID DRIVETRAIN_ANG_PID_GAINS = new PID(0, 0.009, 0, 0.008);

    // Constants
    public static final double DRIVE_ENCODER_COUNTS_PER_REV = 4096;

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
}