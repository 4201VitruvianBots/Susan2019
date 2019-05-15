package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Pneumatics extends Subsystem {

    //public DoubleSolenoid testPiston = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.testPistonForward, RobotMap.testPistonReverse);

    public Pneumatics(){
        super("Pneumatics");

    }

//    public boolean getTestPistonStatus(){
//        return (testPiston.get() == DoubleSolenoid.Value.kForward) ? true : false;
//    }

    public void setTestPiston(boolean state) {
      //  testPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    @Override
    protected void initDefaultCommand() {

    }
}
