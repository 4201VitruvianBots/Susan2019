package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import frc.vitruvianlib.driverstation.Shuffleboard;
import frc.vitruvianlib.util.PIDOutputInterface;

public class AlignToTarget extends Command {
    static double kP = 0.03;
    static double kI = 0;
    static double kD = 0;
    static double outputMagnitude = 0.6;
    double setpoint = 0;

    public AlignToTarget() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        requires(Robot.limelight);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.resetAngle();

        kP = Shuffleboard.getNumber("LimelightPID", "kP", kP);
        kI = Shuffleboard.getNumber("LimelightPID", "kI", kI);
        kD = Shuffleboard.getNumber("LimelightPID", "kD", kD);
        outputMagnitude = Shuffleboard.getNumber("LimelightPID", "output", outputMagnitude);

        Robot.driveTrain.limelightPIDController.setP(kP);
        Robot.driveTrain.limelightPIDController.setI(kI);
        Robot.driveTrain.limelightPIDController.setD(kD);
        Robot.driveTrain.limelightPIDController.setOutputRange(-outputMagnitude, outputMagnitude);

        if(Robot.limelight.isValidTarget())
            Robot.driveTrain.limelightPIDController.enable();
        else
            System.out.println("Error: No targets detected");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
            setpoint = Robot.limelight.getTargetX();
            Robot.driveTrain.limelightPIDController.setSetpoint(setpoint);

            Shuffleboard.putNumber("LimelightPID", "setpoint", setpoint);

            double output =  Robot.driveTrain.limelightPIDOutput.getPIDOutput();
            Robot.driveTrain.driveTank(output, -output);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.driveTrain.limelightPIDController.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.limelightPIDController.disable();
        Robot.driveTrain.driveArcade(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
