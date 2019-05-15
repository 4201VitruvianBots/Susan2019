/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.												*/
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 * Team 4201 RobotMap Standards
 * We have a standard mapping scheme as a baseline to make our basic code 
 * portable to new robots every year. This also allows us to ensure our
 * basic wiring is correct. 
 * 
 * For each address value, the left/positive value should always be an even
 * number if it is a paired value. For non-paired values, skip the next odd
 * number.
 * 
 * CAN ADDRESSES
 * 0-19: Major Modules
 * -> 0: PDP
 * -> 1-9: VRMs
 * -> 11-19: PCMs
 * 20-59: Motor Controllers
 * -> 20-29: DriveTrain Motors
 * -> 30-39: MechanismA
 * -> 40-49: MechanismB
 * -> 50-59: MechanismC
 * 
 */
public class RobotMap {
	// GLOBAL VARIABLES
	public static final int leftJoystick = 0;
	public static final int rightJoystick = 1;
	public static final int xBoxController = 2;
	
	// Joystick Constants
	public static final int leftTrigger = 2;
	public static final int rightTrigger = 3;

	// Electrical Modules
	public static final int PDP = 0;

	// Pneumatic Modules & their devices
	public static final int PCMOne = 11;
	public static final int driveTrainShifterForward = 0;
	public static final int driveTrainShifterReverse = 1;
	public static final int testPistonForward = 0;
	public static final int testPistonReverse = 1;

	// Motor Controllers
	public static final int driveTrainLeftMaster = 21;
	public static final int driveTrainLeftSlave = 20;
	public static final int driveTrainRightMaster = 22;
	public static final int driveTrainRightSlave = 23;
	public static final int testSparkMAX = 20;

	// Bling
	public static final int LEDCh1 = 23;	//pins on the navX, 23 is 9, 24 is 8, 10 is 0, 11 is 1, etc.
	public static final int LEDCh2 = 22;	//should have more general variable names, someone refactor this
	public static final int LEDCh3 = 21;
	public static final int LEDCh4 = 20;
	public static final int LimSwitch = 19;		//Used for testing onboard sensors
	public static final int SelfAddress = 16;	//used for I2C communication
	public static final int ArduinoAddress = 8;


}
