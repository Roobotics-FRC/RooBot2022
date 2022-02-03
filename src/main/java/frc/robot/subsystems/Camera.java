package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private static volatile Camera instance;

    /**
     * The getter for the Camera class.
     * @return the singleton Camera object.
     */
    public static Camera getInstance() {
        if (instance == null) {
            synchronized (Camera.class) {
                if (instance == null) {
                    instance = new Camera();
                }
            }
        }
        return instance;
    }

    private Camera() {

    }

    public void processFrame() {

    }

    public void logToSmartDashboard() {

    }
}