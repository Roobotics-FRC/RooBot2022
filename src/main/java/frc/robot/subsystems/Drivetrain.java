package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotMap;

public class Drivetrain extends PIDSubsystem {
    private static volatile Drivetrain instance;

    private TalonFX right1;
    private TalonFX right2;
    private TalonFX right3;
    private TalonFX left1;
    private TalonFX left2;
    private TalonFX left3;

    private PigeonIMU pigeon;
    private double initialAngle;

    static final double pid_p = 0;
    static final double pid_i = 0;
    static final double pid_d = 0;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            synchronized (Drivetrain.class) {
                if (instance == null) {
                    instance = new Drivetrain();
                }
            }
        }
        return instance;
    }

    private Drivetrain() {
        super(new PIDController(RobotMap.DRIVETRAIN_ANG_PID_GAINS.kP, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kI, RobotMap.DRIVETRAIN_ANG_PID_GAINS.kD));
        getController().setTolerance(0.1, 0.1);

        this.right1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1.id);
        this.right2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2.id);
        this.right3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_3.id);
        this.left1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_1.id);
        this.left2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_2.id);
        this.left3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_3.id);

        setNeutralMode(NeutralMode.Brake);

        this.right1.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1.inverted);
        this.right2.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2.inverted);
        this.right3.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_3.inverted);
        
        this.left1.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_1.inverted);
        this.left2.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_2.inverted);
        this.left3.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_3.inverted);  

        this.right2.follow(right1);
        this.right3.follow(right1);
        this.left2.follow(left1);
        this.left3.follow(left1);

        this.right1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.right1.setSensorPhase(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1.encoderPhase);
        this.left1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.left1.setSensorPhase(RobotMap.DRIVETRAIN_MOTOR_LEFT_1.encoderPhase);

        right1.config_kF(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kF);
        right1.config_kP(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kP);
        right1.config_kI(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kI);
        right1.config_kD(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kD);

        left1.config_kF(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kF);
        left1.config_kP(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kP);
        left1.config_kI(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kI);
        left1.config_kD(0, RobotMap.DRIVETRAIN_TALON_PID_GAINS.kD);

        pigeon = new PigeonIMU(RobotMap.PIGEON_ID);

        getController().setTolerance(5, 10);
        getController().enableContinuousInput(0, 360);
    }

    public double getPigeonAngle() {
        return normalizeAngle(this.getPigeonYawRaw() - this.initialAngle);
    }

    private double normalizeAngle(double angle) {
        return leastResidue(angle, 360.0D);
    }

    private double leastResidue(double n, double modulus) throws IllegalArgumentException {
        if (modulus <= 0.0D) {
            throw new IllegalArgumentException("Modulus cannot be less than or equal to zero.");
        } else {
            return (n % modulus + modulus) % modulus;
        }
    }

    public void resetPigeonYaw() {
        this.initialAngle = this.getPigeonYawRaw();
    }

    public void setPigeonYaw(double angle) {
        this.initialAngle = this.getPigeonYawRaw() + angle;
    }

    private double getPigeonYawRaw() {
        double[] ypr = new double[3];
        this.pigeon.getYawPitchRoll(ypr);
        return ypr[0] * -1.0D;
    }

    @Override
    protected double getMeasurement() {
        return getPigeonAngle();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double outputConstrained = RobotMap.pidConstrainPercentOutput(output);
        setRightPercentOutput(outputConstrained);
        setLeftPercentOutput(-outputConstrained);
        SmartDashboard.putNumber("DrivePIDOutput", outputConstrained);
        SmartDashboard.putNumber("DrivePIDSetPoint", setpoint);
        
    }

    public void setNeutralMode(NeutralMode mode) {
        this.right1.setNeutralMode(mode);
        this.right2.setNeutralMode(mode);
        this.right3.setNeutralMode(mode);
        this.left1.setNeutralMode(mode);
        this.left2.setNeutralMode(mode);
        this.left3.setNeutralMode(mode);
    }

    public void setRightPercentOutput(double speed) {
        right1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        right2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        right3.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setLeftPercentOutput(double speed) {
        left1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        left2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        left3.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setRightVelocity(double speed) {
        right1.set(ControlMode.Velocity, speed);
    }

    public void setLeftVelocity(double speed) {
        left1.set(ControlMode.Velocity, speed);
    }

    public double getRightPositionInches() {
        return (right1.getSelectedSensorPosition() * ((RobotMap.DRIVE_WHEEL_CIRCUMFRENCE)/(RobotMap.DRIVE_COUNTS_PER_REV * RobotMap.DRIVE_GEAR_RATIO)));
    }

    public double getLeftPositionInches() {
        return (left1.getSelectedSensorPosition() * ((RobotMap.DRIVE_WHEEL_CIRCUMFRENCE)/(RobotMap.DRIVE_COUNTS_PER_REV * RobotMap.DRIVE_GEAR_RATIO)));
    }

    public void setDistanceRight(double position) {
        double encoderUnits = position * ((RobotMap.DRIVE_COUNTS_PER_REV * RobotMap.DRIVE_GEAR_RATIO) / (RobotMap.DRIVE_WHEEL_CIRCUMFRENCE));
        right1.set(ControlMode.Position, encoderUnits);
    }

    public void setDistanceLeft(double position) {
        double encoderUnits = position * ((RobotMap.DRIVE_COUNTS_PER_REV * RobotMap.DRIVE_GEAR_RATIO) / (RobotMap.DRIVE_WHEEL_CIRCUMFRENCE));
        left1.set(ControlMode.Position, encoderUnits);
    }

    public void stop() {
        left1.set(ControlMode.PercentOutput, 0);
        left2.set(ControlMode.PercentOutput, 0);
        left3.set(ControlMode.PercentOutput, 0);

        right1.set(ControlMode.PercentOutput, 0);
        right2.set(ControlMode.PercentOutput, 0);
        right3.set(ControlMode.PercentOutput, 0);
    }
}