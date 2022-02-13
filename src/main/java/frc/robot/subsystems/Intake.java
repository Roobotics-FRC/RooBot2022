package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private static volatile Intake instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Intake getInstance() {
        if (instance == null) {
            synchronized (Intake.class) {
                if (instance == null) {
                    instance = new Intake();
                }
            }
        }
        return instance;
    }

    private Intake() {
        // motor1 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_1.id);
        // motor2 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_2.id);
        // motor2.follow(motor1);

        // motor1.setNeutralMode(NeutralMode.Coast);
        // motor2.setNeutralMode(NeutralMode.Coast);

        // motor1.setInverted(RobotMap.INTAKE_MOTOR_1.inverted);
        // motor2.setInverted(RobotMap.INTAKE_MOTOR_2.inverted);
    }

    public void startIntake() {
        motor1.set(ControlMode.PercentOutput, 0.5);
        motor2.set(ControlMode.PercentOutput, 0.5);
    }

    public void stopIntake() {
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
}