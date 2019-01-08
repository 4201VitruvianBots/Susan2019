package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.vitruvianlib.driverstation.Shuffleboard;
import frc.vitruvianlib.util.PIDOutputInterface;

public class AlignToTargetDavid extends Command {
    //PIDController driveGyroPIDController;
    //PIDOutputInterface driveTurnPIDOutput;
    static double kP = 0.3;        		// Start with P = 10% of your max output, double until you get a quarter-decay oscillation
    static double kI = 0;           // Don't use I
    static double kD = 0.01;           	// Start with D = P * 10
    private double kS = 0.5; //Voltage to break static friction
    private double kV = 0.5; //Voltage to hold constant velocity
    private double kA = 0.5; //Voltage to hold constant acceleration
    static double period = 0.01;
    //static double outputMagnitude = 0;
    double setpoint = 0.6;
    double PIDOutput = 0;
    double ClosedLoopOutput = 0;
    private double PIDPreviousTime;
    private double PIDPreviousError;
    private Timer PIDTimer;
    private double maxTurnSpeed = 5;
    private double maxAcceleration = 3;

    public AlignToTargetDavid() {
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
        maxTurnSpeed = Shuffleboard.getNumber("DriveTrainConstants", "maxTurnSpeed", maxTurnSpeed);
        maxAcceleration = Shuffleboard.getNumber("DriveTrainConstants", "maxAcceleration", maxAcceleration);
        //outputMagnitude = Shuffleboard.getNumber("LimelightPID", "output", outputMagnitude);
        setpoint = Robot.limelight.getTargetX();





        /*driveTurnPIDOutput = new PIDOutputInterface();
        driveGyroPIDController = new PIDController(kP, kI, kD, Robot.driveTrain.navX, driveTurnPIDOutput, period);
        driveGyroPIDController.setName("Drive Gyro PID");
        driveGyroPIDController.setSubsystem("Drive Train");
        driveGyroPIDController.setAbsoluteTolerance(1.5);
        driveGyroPIDController.setOutputRange(-outputMagnitude, outputMagnitude);
        driveGyroPIDController.enable();*/

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Robot.limelight.isValidTarget()) {
            double PIDSetpoint = Robot.limelight.getTargetX();
            //driveGyroPIDController.setSetpoint(setpoint);

            //PID(feed backward loop)
            double PIDTurnSpeed = (PIDSetpoint-PIDPreviousError)/(PIDPreviousTime-PIDTimer.getFPGATimestamp());
            double PIDVoltage = 0;
            double PIDError = PIDSetpoint-Robot.driveTrain.getAngle();
            PIDVoltage = kP*PIDError+kD*(PIDError-PIDPreviousError)/(PIDPreviousTime-PIDTimer.getFPGATimestamp()-PIDTurnSpeed);
            PIDPreviousError = PIDError;
            PIDPreviousTime = PIDTimer.getFPGATimestamp();


            //Feed forward loop
            double FFVoltage = 0;
            double FFError = setpoint-Robot.driveTrain.getAngle();
            double FFTurnSpeed = (setpoint-PIDPreviousError)/(PIDPreviousTime-PIDTimer.getFPGATimestamp());
            if (FFTurnSpeed <= maxTurnSpeed && FFError >= FFTurnSpeed*FFTurnSpeed/(2*(maxAcceleration))) {
                FFVoltage = kS + kV * FFTurnSpeed + kA * maxAcceleration;
            } else if (FFError >= FFTurnSpeed*FFTurnSpeed/(2*(maxAcceleration))){
                FFVoltage = kS + kV * FFTurnSpeed + kA * -maxAcceleration;
            } else {
                FFVoltage = kS + kV * FFTurnSpeed;
            }

            //Uses voltages
            double output = FFVoltage + PIDVoltage;
            if (output > 1)
                output=1;

            Robot.driveTrain.driveTank(-output, output);
        } else
            System.out.println("Error: No targets detected");
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        //return driveGyroPIDController.onTarget();
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        //driveGyroPIDController.disable();
        Robot.driveTrain.driveArcade(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
