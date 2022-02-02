package frc.robot.input;


import frc.robot.RobotMap;
import frc.robot.input.filters.DummyFilter;
import frc.robot.input.filters.LogitechFilter;

/**
 * OI provides access to operator interface devices.
 */
public final class OI {
    private static volatile OI oi = null;
    private RooJoystick driveJoystick;
    private RooJoystick operatorJoystick;

    private OI() {
        this.driveJoystick = new RooJoystick(RobotMap.DRIVE_JOYSTICK_PORT,
                new LogitechFilter(), 0.1);
        driveJoystick.configureAxis(driveJoystick.getZChannel(),
                new LogitechFilter(), 0.1);
        driveJoystick.configureAxis(driveJoystick.getThrottleChannel(),
                new DummyFilter(), 0);
        this.operatorJoystick = new RooJoystick(RobotMap.OPERATOR_JOYSTICK_PORT,
                new DummyFilter(), 0);
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
