package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static volatile Drivetrain instance;

    private TalonFX right1;
    private TalonFX right2;
    private TalonFX right3;
    private TalonFX left1;
    private TalonFX left2;
    private TalonFX left3;

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
        // setDefaultCommand(new DriveWithJoystick());

        this.right1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_1);
        this.right2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_2);
        this.right3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_RIGHT_3);
        this.left1 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_1);
        this.left2 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_2);
        this.left3 = new WPI_TalonFX(RobotMap.DRIVETRAIN_MOTOR_LEFT_3);

        setNeutralMode(NeutralMode.Coast);

        this.right1.setInverted(true);
        this.right2.setInverted(true);
        this.right3.setInverted(true);

        this.right2.follow(right1);
        this.right3.follow(right1);
        this.left2.follow(left1);
        this.left3.follow(left1);

        this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.right1.setSensorPhase(RobotMap.DRIVETRAIN_RIGHT_ENCODER_PHASE);
        this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.left1.setSensorPhase(RobotMap.DRIVETRAIN_LEFT_ENCODER_PHASE);
    }

    public void setNeutralMode(NeutralMode mode) {
        this.right1.setNeutralMode(mode);
        this.right2.setNeutralMode(mode);
        this.right3.setNeutralMode(mode);
        this.left1.setNeutralMode(mode);
        this.left2.setNeutralMode(mode);
        this.left3.setNeutralMode(mode);
    }

    public void setRight(double speed) {
        right1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        right2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        right3.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void setLeft(double speed) {
        left1.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        left2.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
        left3.set(ControlMode.PercentOutput, RobotMap.constrainPercentOutput(speed));
    }

    public void stop() {
        left1.set(ControlMode.PercentOutput, 0);
        left2.set(ControlMode.PercentOutput, 0);
        left3.set(ControlMode.PercentOutput, 0);

        right1.set(ControlMode.PercentOutput, 0);
        right2.set(ControlMode.PercentOutput, 0);
        right3.set(ControlMode.PercentOutput, 0);
    }

    public void logToSmartDashboard() {

    }
}