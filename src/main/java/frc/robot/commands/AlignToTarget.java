package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.vitruvianlib.driverstation.Shuffleboard;
import frc.vitruvianlib.util.PIDOutputInterface;

public class AlignToTarget extends Command {
    PIDController driveGyroPIDController;
    PIDOutputInterface driveTurnPIDOutput;
    static double kP = 0.3;        		// Start with P = 10% of your max output, double until you get a quarter-decay oscillation
    static double kI = 0;           // Start with I = P / 100
    static double kD = 0.01;           	// Start with D = P * 10
    static double period = 0.01;
    static double outputMagnitude = 0;
    double setpoint = 0.6;

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

        driveTurnPIDOutput = new PIDOutputInterface();
        driveGyroPIDController = new PIDController(kP, kI, kD, Robot.driveTrain.navX, driveTurnPIDOutput, period);
        driveGyroPIDController.setName("Drive Gyro PID");
        driveGyroPIDController.setSubsystem("Drive Train");
        driveGyroPIDController.setAbsoluteTolerance(1.5);
        driveGyroPIDController.setOutputRange(-outputMagnitude, outputMagnitude);
        driveGyroPIDController.enable();

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.limelight.isValidTarget()) {
            setpoint = Robot.limelight.getTargetX();
            driveGyroPIDController.setSetpoint(setpoint);

            double output = driveTurnPIDOutput.getPIDOutput();
            Robot.driveTrain.driveTank(output, -output);
        } else
            System.out.println("Error: No targets detected");
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return driveGyroPIDController.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        driveGyroPIDController.disable();
        Robot.driveTrain.driveArcade(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
