package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static volatile Drivetrain instance;

    private WPI_TalonFX right1;
    private WPI_TalonFX right2;
    private WPI_TalonFX right3;
    private WPI_TalonFX left1;
    private WPI_TalonFX left2;
    private WPI_TalonFX left3;

    private DifferentialDrive tankDrive;

    private DifferentialDriveOdometry tankOdometry;

    private PigeonIMU pigeon;

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
        this.right1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1.id);
        this.right2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2.id);
        this.right3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_3.id);
        MotorControllerGroup rightGroup = new MotorControllerGroup(right1, right2, right3);

        this.left1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_1.id);
        this.left2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_2.id);
        this.left3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_3.id);
        MotorControllerGroup leftGroup = new MotorControllerGroup(left1, left2, left3);

        tankDrive = new DifferentialDrive(leftGroup, rightGroup);

        setNeutralMode(NeutralMode.Brake);

        right2.follow(right1);
        right3.follow(right1);

        left2.follow(left1);
        left3.follow(left1);

        this.right1.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1.inverted);
        this.right2.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2.inverted);
        this.right3.setInverted(RobotMap.DRIVETRAIN_MOTOR_RIGHT_3.inverted);
        
        this.left1.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_1.inverted);
        this.left2.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_2.inverted);
        this.left3.setInverted(RobotMap.DRIVETRAIN_MOTOR_LEFT_3.inverted);  

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
        tankOdometry = new DifferentialDriveOdometry(new Rotation2d(Units.degreesToRadians(getPigeonAngle())), new Pose2d(0, 0, new Rotation2d(0)));
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public void setMotorVoltage(double leftVolts, double rightVolts) {
        right1.setVoltage(rightVolts);
        right2.setVoltage(rightVolts);
        right3.setVoltage(rightVolts);            
        left1.setVoltage(leftVolts);
        left2.setVoltage(leftVolts);
        left3.setVoltage(leftVolts);
        tankDrive.feed();
    }

    public Pose2d getPose() {
        return tankOdometry.getPoseMeters();
    }

    public void resetEncoders() {
        left1.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        tankOdometry.resetPosition(pose, new Rotation2d(Units.degreesToRadians(getPigeonAngle())));
    }

    public void updateOdometry() {
        tankOdometry.update(new Rotation2d(Units.degreesToRadians(getPigeonAngle())), Units.inchesToMeters(getRightPositionInches()), Units.inchesToMeters(getLeftPositionInches()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getRightVelocity(),getLeftVelocity());
    }

    public void setWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wSpeeds = RobotMap.DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds);
        drive(wSpeeds.leftMetersPerSecond, wSpeeds.rightMetersPerSecond);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return RobotMap.DRIVE_KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getRightVelocity(),getLeftVelocity()));
    }

    public double getRightVelocity() {
        return Units.inchesToMeters((right1.getSelectedSensorVelocity() / 2048) * (Math.PI * 6));
    }

    // Returns in METERS PER SECOND
    public double getLeftVelocity() {
        // 2048 encoder units per rotation
        // Wheel diameter of 6 inches
        // Wheel circumfrence of 12pi
        return Units.inchesToMeters((left1.getSelectedSensorVelocity() / 2048) * (Math.PI * 6));
    }

    public void drive(double leftSpeed, double rightSpeed) {
        tankDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public double getPigeonAngle() {
        // return normalizeAngle(this.getPigeonYawRaw() - this.initialAngle);
        return normalizeAngle(this.getPigeonYawRaw());
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

    private double getPigeonYawRaw() {
        double[] ypr = new double[3];
        this.pigeon.getYawPitchRoll(ypr);
        return ypr[0] * -1.0D;
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