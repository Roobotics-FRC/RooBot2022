package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb extends SubsystemBase {
    private static volatile Climb instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

    private DoubleSolenoid armSolenoid;

    /**
     * The getter for the Drivetrain class.
     * @return the singleton Drivetrain object.
     */
    public static Climb getInstance() {
        if (instance == null) {
            synchronized (Climb.class) {
                if (instance == null) {
                    instance = new Climb();
                }
            }
        }
        return instance;
    }

    private Climb() {
        this.motor1 = new WPI_TalonSRX(RobotMap.CLIMB_MOTOR_1.id);
        this.motor2 = new WPI_TalonSRX(RobotMap.CLIMB_MOTOR_2.id);

        this.motor1.setNeutralMode(NeutralMode.Brake);
        this.motor2.setNeutralMode(NeutralMode.Brake);

        this.motor1.setInverted(RobotMap.CLIMB_MOTOR_1.inverted);
        this.motor2.setInverted(RobotMap.CLIMB_MOTOR_2.inverted);

        armSolenoid = new DoubleSolenoid(RobotMap.PCM_PORT, PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_ARM_SOLENOID_DEPLOY, RobotMap.CLIMB_ARM_SOLENOID_RETRACT);
    }

    public void setPercentOutput(double speed) {
        motor1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        motor2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setClimb(double speed) {
        setPercentOutput(speed);
    }

    public boolean armsDeployed() {
        return armSolenoid.get() == RobotMap.DEPLOYED;
    }

    public void deployArms() {
        armSolenoid.set(RobotMap.DEPLOYED);
    }

    public void retractArms() {
        armSolenoid.set(RobotMap.RETRACTED);
    }

    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
}
