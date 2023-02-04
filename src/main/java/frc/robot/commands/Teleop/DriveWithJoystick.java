package frc.robot.commands.Teleop;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.input.OI;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends CommandBase {
    public Drivetrain drivetrain;
    private boolean recording = false;
    private boolean running = false;
    private int i;
    private List<Double> rightVoltages = new ArrayList<Double>();
    private List<Double> leftVoltages = new ArrayList<Double>();

    public DriveWithJoystick() {
        addRequirements(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void initialize() {
        drivetrain.stop();
    }

    @Override
    public void execute() {
        drivetrain.drive(OI.getInstance().getCyleController().getRawAxis(RobotMap.DRIVE_LEFT_AXIS), OI.getInstance().getCyleController().getRawAxis(RobotMap.DRIVE_RIGHT_AXIS));
        if (recording) {
            rightVoltages.add(drivetrain.getRightVoltage());
            leftVoltages.add(drivetrain.getRightVoltage());
        }
        if (running) {
            if (i < leftVoltages.size()) {
                drivetrain.setMotorVoltage(leftVoltages.get(i), rightVoltages.get(i));
                i += 1;
            } else {
                running = false;
            }
        }
        if (OI.getInstance().getCyleController().getRawButtonPressed(14)) {
            if (recording) {
                recording = false;
            } else {
                recording = true;
                running = false;
                if (leftVoltages.size() > 0) {
                    rightVoltages.clear();
                    leftVoltages.clear();
                }
            }
        }
        if (OI.getInstance().getCyleController().getRawButtonPressed(15)) {
            running = !running;
            recording = false;
            i = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}