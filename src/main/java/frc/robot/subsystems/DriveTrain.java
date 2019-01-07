
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.SetArcadeDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
	public WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(RobotMap.driveTrainLeftMaster);
	public WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(RobotMap.driveTrainLeftSlave);
	public WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.driveTrainRightMaster);
	public WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(RobotMap.driveTrainRightSlave);

	DifferentialDrive robotDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

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

	public void initDefaultCommand() {
		setDefaultCommand(new SetArcadeDrive());
	}
}