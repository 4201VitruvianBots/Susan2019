/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.												*/
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.vitruvianlib.driverstation.Shuffleboard;

public class Robot extends TimedRobot {
	public static DriveTrain driveTrain = new DriveTrain();
	public static Limelight limelight = new Limelight();
	public static Pneumatics pneumatics = new Pneumatics();
	public static OI oi;

	Command m_teleOpCommand;
	Command m_autonomousCommand;

	@Override
	public void robotInit() {
		oi = new OI();

		Shuffleboard.putNumber("LimelightPID", "kP", 0.3);
		Shuffleboard.putNumber("LimelightPID", "kI", 0);
		Shuffleboard.putNumber("LimelightPID", "kD", 0.01);
		Shuffleboard.putNumber("LimelightPID", "output", 0.6);

	}

	@Override
	public void robotPeriodic() {
		driveTrain.updateSmartDashboard();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		//m_autonomousCommand = ;

		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
