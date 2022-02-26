package frc.robot.input;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap;
import frc.robot.commands.Auton.DriveDistanceAuton;
import frc.robot.commands.Teleop.DriveTurnToAngle;
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
    private JoystickButton driveForwardButton;

    private OI() {
        this.cyleController = new RooJoystick(RobotMap.CYCLE_CONTROLLER_PORT, new LogitechFilter(), 0.05);

        this.operatorController = new RooJoystick(RobotMap.OPERATOR_CONTROLLER_PORT,
                new DummyFilter(), 0);
        this.turn90DegreesButton = new JoystickButton(this.cyleController,
                RobotMap.DRIVE_TURN_TO_TARGET_BUTTON);
        this.turn90DegreesButton.whenPressed(new DriveTurnToAngle());

        this.driveForwardButton = new JoystickButton(this.cyleController,
                RobotMap.DRIVE_DISTANCE_BUTTON);
        this.driveForwardButton.whenPressed(new DriveDistanceAuton(100));
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
