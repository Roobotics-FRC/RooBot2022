package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTurnToAngleWithoutVision extends CommandBase {
        public Drivetrain drivetrain;
        private double angle;
        
        public DriveTurnToAngleWithoutVision(double angle) {
            addRequirements(this.drivetrain = Drivetrain.getInstance());
            this.angle = angle;
        }
    
        @Override
        public void initialize() {
            drivetrain.resetPigeonYaw();
            drivetrain.enable();
            drivetrain.setSetpoint(angle);
        }
    
        @Override
        public void execute() {}
    
        @Override
        public void end(boolean interrupted) {
            drivetrain.stop();
            drivetrain.disable();
        }
    
        @Override
        public boolean isFinished() {
            return Math.abs(angle) < 3;
        }
}