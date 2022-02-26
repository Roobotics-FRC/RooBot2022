package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static volatile Shooter instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

    private DoubleSolenoid angleSolenoid;

    private WPI_TalonSRX feederMotor;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Shooter getInstance() {
        if (instance == null) {
            synchronized (Shooter.class) {
                if (instance == null) {
                    instance = new Shooter();
                }
            }
        }
        return instance;
    }

    private Shooter() {
        motor1 = new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_1.id);
        motor2 = new WPI_TalonSRX(RobotMap.SHOOTER_MOTOR_2.id);
        motor2.follow(motor1);

        feederMotor = new WPI_TalonSRX(RobotMap.SHOOTER_FEEDER_MOTOR.id);

        motor1.setNeutralMode(NeutralMode.Coast);
        motor2.setNeutralMode(NeutralMode.Coast);

        feederMotor.setNeutralMode(NeutralMode.Coast);

        motor1.setInverted(RobotMap.SHOOTER_MOTOR_1.inverted);
        motor2.setInverted(RobotMap.SHOOTER_MOTOR_2.inverted);

        feederMotor.setInverted(RobotMap.SHOOTER_FEEDER_MOTOR.inverted);

        motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motor1.setSensorPhase(RobotMap.SHOOTER_MOTOR_1.encoderPhase);

        motor1.config_kF(0, RobotMap.SHOOTER_PID_GAINS.kF);
        motor1.config_kP(0, RobotMap.SHOOTER_PID_GAINS.kP);
        motor1.config_kI(0, RobotMap.SHOOTER_PID_GAINS.kI);
        motor1.config_kD(0, RobotMap.SHOOTER_PID_GAINS.kD);

        angleSolenoid = new DoubleSolenoid(RobotMap.PCM_PORT, PneumaticsModuleType.CTREPCM, RobotMap.SHOOTER_ANGLE_SOLENOID_DEPLOY, RobotMap.SHOOTER_ANGLE_SOLENOID_RETRACT);
    }

    public void setPercentOutput(double speed) {
        motor1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        motor2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setPercentOutput1(double speed) {
        motor1.set(ControlMode.PercentOutput, 1);
    }

    public void setPercentOutput2(double speed) {
        motor2.set(ControlMode.PercentOutput, 1);
    }

    public void setVelocity(double speed) {
        // Max looks like 100,000? Thats what it is in 2020
        motor1.set(ControlMode.Velocity, speed);
    }

    public double getVelocity() {
        return motor1.getSelectedSensorVelocity();
    }

    public void feed() {
        feederMotor.set(ControlMode.PercentOutput, 1);
    }

    public void reverseFeed() {
        feederMotor.set(ControlMode.PercentOutput, -1);
    }

    public void stopFeeder() {
        feederMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setShooterAngled() {
        angleSolenoid.set(RobotMap.DEPLOYED);
    }

    public void setShooterFlat() {
        angleSolenoid.set(RobotMap.RETRACTED);
    }

    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0);
    }
}
