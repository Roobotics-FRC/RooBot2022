package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb extends SubsystemBase {
    private static volatile Climb instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private DigitalInput bottomLimitSwitch;
    private DigitalInput topLimitSwitch;

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

        this.bottomLimitSwitch = new DigitalInput(RobotMap.BOTTOM_LIMIT_SWITCH_DIO_PORT);
        this.topLimitSwitch = new DigitalInput(RobotMap.TOP_LIMIT_SWITCH_DIO_PORT);
    }

    public void setPercentOutput(double speed) {
        motor1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        motor2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setClimb(double speed) {
        if (speed > 0) {
            if (topLimitSwitch.get()) {
                setPercentOutput(speed);
            }
        } else {
            if (bottomLimitSwitch.get()) {
                setPercentOutput(speed);
            }
        }
    }

    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
}
