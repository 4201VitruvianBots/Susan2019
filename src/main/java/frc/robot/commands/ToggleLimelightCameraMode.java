package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ToggleLimelightCameraMode extends InstantCommand {
    public ToggleLimelightCameraMode() {
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        int camMode = Robot.limelight.getCameraMode();

        if(camMode == 0)
            Robot.limelight.setCameraMode(1);
        else if(camMode == 1)
            Robot.limelight.setCameraMode(0);
        else
            System.out.println("Error: Limelight not detected!");

    }

    // Called once after isFinished returns true
    @Override
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
