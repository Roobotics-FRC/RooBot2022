package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static volatile Shooter instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

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

        motor2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motor2.setSensorPhase(RobotMap.SHOOTER_MOTOR_1.encoderPhase);
    }

    public void setSpeed(double speed) {
        motor1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        motor2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public double getVelocity() {
        return motor2.getSelectedSensorVelocity();
    }

    public void feed() {
        feederMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void stopFeeder() {
        feederMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
}
