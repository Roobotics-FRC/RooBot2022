package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb extends SubsystemBase {
    private static volatile Climb instance;

    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;

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
        motor1 = new WPI_TalonSRX(RobotMap.CLIMB_MOTOR_1.id);
        motor2 = new WPI_TalonSRX(RobotMap.CLIMB_MOTOR_2.id);
        motor2.follow(motor1);

        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);

        motor1.setInverted(RobotMap.CLIMB_MOTOR_1.inverted);
        motor2.setInverted(RobotMap.CLIMB_MOTOR_2.inverted);
    }

    public void setPercentOutput(double speed) {
        motor1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void raiseClimb() {
        setPercentOutput(0.5);
    }

    public void lowerClimb() {
        setPercentOutput(-0.5);
    }

    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0);
    }
}
