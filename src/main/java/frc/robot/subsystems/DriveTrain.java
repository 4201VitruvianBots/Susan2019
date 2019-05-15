
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.SetTankDrive;
import frc.vitruvianlib.driverstation.Shuffleboard;
import frc.vitruvianlib.util.PIDOutputInterface;
import com.revrobotics.CANSparkMax;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
	private TalonSRX m_leftMaster = new TalonSRX(RobotMap.driveTrainLeftMaster);
	private TalonSRX m_rightMaster = new TalonSRX(RobotMap.driveTrainRightMaster);
	private TalonSRX m_rightSlave = new TalonSRX(RobotMap.driveTrainRightSlave);
	private CANSparkMax m_testNEO = new CANSparkMax(RobotMap.driveTrainLeftSlave, CANSparkMaxLowLevel.MotorType.kBrushless); {
	}

	static double kP = 0.03;        		// Start with P = 10% of your max output, double until you get a quarter-decay oscillation
	static double kI = 0;           // Start with I = P / 100
	static double kD = 0;           	// Start with D = P * 10
	static double period = 0.01;

	public AHRS navX = new AHRS(SPI.Port.kMXP);

	public PIDOutputInterface limelightPIDOutput = new PIDOutputInterface();
	public PIDController limelightPIDController = new PIDController(kP, kI, kD, navX, limelightPIDOutput);

	public DriveTrain(){
		super("Drive Train");

		m_leftMaster.configFactoryDefault();
		m_rightSlave.set(ControlMode.Follower, m_rightMaster.getDeviceID());

		navX.reset();
		navX.zeroYaw();
		limelightPIDController.setName("Drive Train", "Limelight PID");
		limelightPIDController.setAbsoluteTolerance(1.5);
	}
	// Put methods for controlling the subsystem here. Call these from Commands.

	public void driveTank(double left, double right) {

		m_leftMaster.set(ControlMode.PercentOutput,left);
		m_testNEO.set(right);
	}

	public void driveArcade(double throttle, double turn) {
		m_leftMaster.set(ControlMode.PercentOutput,(throttle + turn));
		m_testNEO.set(throttle - turn);

	}

	public double getAngle(){
		// Angle is negated due to that navX being upside-down on Susan
		return -navX.getAngle();
	}

	public double getYaw() {
		return navX.getYaw();
	}

	public double getRoll(){
		return navX.getRoll();
	}

	public double getPitch(){
		return navX.getPitch();
	}

	public void resetAngle(){
		navX.reset();
	}
	
	public void updateSmartDashboard() {
		Shuffleboard.putNumber("LimelightPID", "Angle", getAngle());
		Shuffleboard.putNumber("LimelightPID", "Pitch", getPitch());
		Shuffleboard.putNumber("LimelightPID", "Yaw", getYaw());
		Shuffleboard.putNumber("LimelightPID", "Roll", getRoll());
	}

	public void initDefaultCommand() {
		setDefaultCommand(new SetTankDrive());
	}
}
