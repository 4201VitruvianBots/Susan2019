/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.												*/
/* Open Source Software - may be modified and shared by FRC teams. The code	 */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.																															 */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	public Joystick leftJoystick, rightJoystick;
	public Button[] leftButtons;
	public Button[] rightButtons;
	
	public OI(){
		leftJoystick = new Joystick(RobotMap.leftJoystick);
		rightJoystick = new Joystick(RobotMap.rightJoystick);

		initializeButtons();
	}

	/**
	 * Initializes the buttons on the joysticks/controllers. This is in its own
	 * function in the event we want to have different driver button mappings
	 * between drivers, controls testing, etc.
	 */
	public void initializeButtons() {
		leftButtons = new Button[7];
		rightButtons = new Button[12];
		for(int i = 0; i < leftButtons.length; i++)
			leftButtons[i] = new JoystickButton(leftJoystick, i + 1);
		for(int i = 0; i < rightButtons.length; i++)
			rightButtons[i] = new JoystickButton(rightJoystick, i + 1);

		//leftButtons[1].whenPressed(new ToggleLimelightCameraMode());

		rightButtons[0].whileHeld(new AlignToTarget());
		rightButtons[1].whenPressed(new ToggleTestPiston());
	}
	
	public double getLeftY(){
		return -leftJoystick.getY();
	}

	public double getLeftX(){
		return leftJoystick.getX();
	}

 	public double getRightY(){
		return -rightJoystick.getY();
	}

	public double getRightX(){
		return rightJoystick.getX();
	}
}
