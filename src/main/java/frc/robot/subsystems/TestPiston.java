package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TogglePiston;

public class TestPiston extends Subsystem {

    DoubleSolenoid testPiston;

    public TestPiston(){
        super("TestPiston");

        testPiston = new DoubleSolenoid(RobotMap.PCMOne, RobotMap.testPistonForward, RobotMap.testPistonReverse);
    }

    public void activiateTestPiston(){
        testPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void retractTestPiston(){
        testPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean getTestPistonStatus(){
        return testPiston.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void initDefaultCommand(){
        setDefaultCommand(new TogglePiston());
    }
}
