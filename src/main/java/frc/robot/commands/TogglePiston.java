package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TogglePiston extends Command {
    public TogglePiston() {
        requires(Robot.testPiston);
    }
    public void initialize(){
        if (Robot.testPiston.getTestPistonStatus() == true){
            Robot.testPiston.retractTestPiston();
        } else {
            Robot.testPiston.activiateTestPiston();
        }
    }
    public void execute(){

    }
    public boolean isFinished(){
        return true;
    }
    public void end(){

    }
    public void interupted(){

    }
}
