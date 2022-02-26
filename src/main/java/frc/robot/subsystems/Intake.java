package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private static volatile Intake instance;

    private WPI_TalonSRX motor1;
    private DoubleSolenoid deploySolenoid;

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

        deploySolenoid = new DoubleSolenoid(RobotMap.PCM_PORT, PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_DEPLOY_SOLENOID_DEPLOY, RobotMap.INTAKE_DEPLOY_SOLENOID_RETRACT);
    }

    public void startIntake() {
        motor1.set(ControlMode.PercentOutput, 1);
    }

    public void reverseIntake() {
        motor1.set(ControlMode.PercentOutput, -1);
    }

    public void stopIntake() {
        motor1.set(ControlMode.PercentOutput, 0);
    }

    public void deployIntake() {
        deploySolenoid.set(RobotMap.DEPLOYED);
    }

    public void retractIntake() {
        deploySolenoid.set(RobotMap.RETRACTED);
    }
}