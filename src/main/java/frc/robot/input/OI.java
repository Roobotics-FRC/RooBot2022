package frc.robot.input;


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
    private RooJoystick driveJoystick;
    private RooJoystick operatorJoystick;
    private JoystickButton turn90DegreesButton;

    private OI() {
        this.driveJoystick = new RooJoystick(RobotMap.DRIVE_JOYSTICK_PORT,
                new LogitechFilter(), 0.1);
        driveJoystick.configureAxis(driveJoystick.getZChannel(),
                new LogitechFilter(), 0.1);
        driveJoystick.configureAxis(driveJoystick.getThrottleChannel(),
                new DummyFilter(), 0);
        this.operatorJoystick = new RooJoystick(RobotMap.OPERATOR_JOYSTICK_PORT,
                new DummyFilter(), 0);
        this.turn90DegreesButton = new JoystickButton(this.driveJoystick,
                RobotMap.DRIVE_TURN_90_BUTTON);
        this.turn90DegreesButton.whenPressed(new DriveTurnToAngle(90));
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
     * Gets the drive joystick controlling the robot.
     * @return The drive joystick controlling the robot.
     */
    public RooJoystick getDriveJoystick() {
        return this.driveJoystick;
    }

    /**
     * Gets the operator joystick controlling the robot.
     * @return The operator joystick controlling the robot.
     */
    public RooJoystick getOperatorJoystick() {
        return this.operatorJoystick;
    }
}
