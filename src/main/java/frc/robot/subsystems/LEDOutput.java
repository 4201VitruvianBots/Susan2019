/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.I2C;
import frc.robot.RobotMap;
import frc.robot.commands.LimToggle;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LEDOutput extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DigitalOutput[] digitalOutput = {  //array that creates digitalOutput0-4, I think.
        new DigitalOutput(RobotMap.LEDCh1),  //actual pin numbers defined in RobotMap
        new DigitalOutput(RobotMap.LEDCh2),
        new DigitalOutput(RobotMap.LEDCh3),
        new DigitalOutput(RobotMap.LEDCh4),
    };
  private boolean[] DIOState = {
          false,       //initialises variables for all of the pins to false
          false,
          false,
                  false,
};

private DigitalInput digitalInput = new DigitalInput(RobotMap.LimSwitch);  //defines the limit switch as an input
/*
public I2C i2C = new I2C(I2C.Port.kMXP, RobotMap.SelfAddress);        //should be I2C, may be broken and is unused
private int ledStripMode = 0; //initialises the LED strip mode for i2C
*/
  public LEDOutput(){
      super("LED Output");
  }
  public void setPinOutput(boolean state, int pin){
    digitalOutput[pin].set(state);  //uses the digitalOutput to actually write the new state
    DIOState[pin] = state;          //sets our variable to the state so we know what the pin is when we want it below
  }
  public boolean getDIOState(int pin){ return DIOState[pin]; }  //returns the value of the pin, used for toggles & the like
  public boolean getLimState(){return digitalInput.get();}      //returns the value of the limit switch
/*
  public void writeOverI2C(int stripMode) {         //this may be very wrong
    i2C.write(RobotMap.ArduinoAddress, stripMode);
    ledStripMode = stripMode;
  }
  public int getLastSignal() { return ledStripMode;}  //Checks which I2C signal was last sent
*/
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LimToggle());
  }
}
