
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.SetArcadeDrive;
import frc.robot.commands.SetTankDrive;
import frc.vitruvianlib.driverstation.Shuffleboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
	public WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(RobotMap.driveTrainLeftMaster);
	public WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(RobotMap.driveTrainLeftSlave);
	public WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.driveTrainRightMaster);
	public WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(RobotMap.driveTrainRightSlave);

	DifferentialDrive robotDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

	public AHRS navX = new AHRS(SPI.Port.kMXP);
	public DriveTrain(){
		super("Drive Train");

		m_leftSlave.set(ControlMode.Follower, m_leftMaster.getDeviceID());
		m_rightSlave.set(ControlMode.Follower, m_rightMaster.getDeviceID());
	}
	// Put methods for controlling the subsystem here. Call these from Commands.

	public void driveTank(double left, double right) {
		robotDrive.tankDrive(left, right);
	}

	public void driveArcade(double forward, double turn) {
		robotDrive.arcadeDrive(forward, turn);
	}

	public double getAngle(){
		// Angle is negated due to that navX being upside-down on Susan
		return -navX.getAngle();
	}

	public void resetAngle(){
		navX.reset();
	}
	
	public void updateSmartDashboard() {
		Shuffleboard.putNumber("LimelightPID", "Angle", getAngle());
	}

	public void initDefaultCommand() {
		setDefaultCommand(new SetTankDrive());
	}
}
