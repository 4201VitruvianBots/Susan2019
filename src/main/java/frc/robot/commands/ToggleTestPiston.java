package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ToggleTestPiston extends InstantCommand {
    public ToggleTestPiston() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.pneumatics);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
//       if(Robot.pneumatics.getTestPistonStatus())
//           Robot.pneumatics.setTestPiston(false);
//       else
//           Robot.pneumatics.setTestPiston(true);

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
