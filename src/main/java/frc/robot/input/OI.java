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
    private RooJoystick cyleController;
    private RooJoystick operatorController;
    private JoystickButton turn90DegreesButton;

    private OI() {
        this.cyleController = new RooJoystick(RobotMap.CYCLE_CONTROLLER_PORT, new LogitechFilter(), 0.01);

        this.operatorController = new RooJoystick(RobotMap.OPERATOR_CONTROLLER_PORT,
                new DummyFilter(), 0);
        this.turn90DegreesButton = new JoystickButton(this.operatorController,
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
     * Gets the drive controller controlling the robot.
     * @return The drive controller controlling the robot.
     */
    public RooJoystick getCyleController() {
        return this.cyleController;
    }

    /**
     * Gets the operator controller controlling the robot.
     * @return The operator controller controlling the robot.
     */
    public RooJoystick getOperatorController() {
        return this.operatorController;
    }
}
