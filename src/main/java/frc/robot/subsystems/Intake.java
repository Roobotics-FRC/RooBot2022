package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private static volatile Intake instance;

    private WPI_TalonSRX motor1;
    private Solenoid deploySolenoid;

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
        motor1 = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR.id);

        motor1.setNeutralMode(NeutralMode.Coast);

        motor1.setInverted(RobotMap.INTAKE_MOTOR.inverted);

        // deploySolenoid = new Solenoid(moduleType, channel);
    }

    public void startIntake() {
        motor1.set(ControlMode.PercentOutput, 0.5);
    }

    public void stopIntake() {
        motor1.set(ControlMode.PercentOutput, 0);
    }

    public void deployIntake() {
        deploySolenoid.set(true);
    }
}