package frc.robot.input;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.input.filters.DummyFilter;
import frc.robot.input.filters.LogitechFilter;

/**
 * OI provides access to operator interface devices.
 */
public final class OI {
    private static volatile OI oi = null;
    private RooJoystick leftDriveJoystick;
    private RooJoystick rightDriveJoystick;
    private RooJoystick operatorJoystick;
    private JoystickButton turn90DegreesButton;

    private OI() {
        this.leftDriveJoystick = new RooJoystick(RobotMap.LEFT_DRIVE_JOYSTICK_PORT,
                new LogitechFilter(), 0.01);
        leftDriveJoystick.configureAxis(leftDriveJoystick.getZChannel(),
                new LogitechFilter(), 0.01);
        leftDriveJoystick.configureAxis(leftDriveJoystick.getThrottleChannel(),
                new DummyFilter(), 0);

        this.rightDriveJoystick = new RooJoystick(RobotMap.RIGHT_DRIVE_JOYSTICK_PORT,
                new LogitechFilter(), 0.01);
        rightDriveJoystick.configureAxis(rightDriveJoystick.getZChannel(),
                new LogitechFilter(), 0.01);
        rightDriveJoystick.configureAxis(rightDriveJoystick.getThrottleChannel(),
                new DummyFilter(), 0);

        // this.operatorJoystick = new RooJoystick(RobotMap.OPERATOR_JOYSTICK_PORT,
        //         new DummyFilter(), 0);
        this.turn90DegreesButton = new JoystickButton(this.rightDriveJoystick,
                RobotMap.DRIVE_TURN_90_BUTTON);
        this.turn90DegreesButton.whenPressed(new DriveTurnToAngle(90, 100, true));
    }

    /**
     * The getter for the OI singleton.
     *
     * @return The static OI singleton object.
     */
    public static OI getInstance() {
        if (oi == null) {
            synchronized (OI.class) {
                if (oi == null) {
                    oi = new OI();
                }
            }
        }
        return oi;
    }

    /**
     * Gets the left drive joystick controlling the robot.
     * @return The left drive joystick controlling the robot.
     */
    public RooJoystick getLeftDriveJoystick() {
        return this.leftDriveJoystick;
    }

    /**
     * Gets the right drive joystick controlling the robot.
     * @return The right drive joystick controlling the robot.
     */
    public RooJoystick getRightDriveJoystick() {
        return this.rightDriveJoystick;
    }

    /**
     * Gets the operator joystick controlling the robot.
     * @return The operator joystick controlling the robot.
     */
    public RooJoystick getOperatorJoystick() {
        return this.operatorJoystick;
    }
}
