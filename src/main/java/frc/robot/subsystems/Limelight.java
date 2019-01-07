package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends Subsystem {
    static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(){
        super("Limelight");
    }

    public int getCameraMode() {
        return (int) limelightTable.getEntry("camMode").getDouble(0);
    }

    public void setCameraMode(int camMode) {
        limelightTable.getEntry("camMode").setNumber(camMode);
    }

    public double getTargetX(){
        return  limelightTable.getEntry("tx").getDouble(0);
    }

    public boolean isValidTarget(){
        return (limelightTable.getEntry("tv").getDouble(0) == 1) ? true : false;
    }

    public void initDefaultCommand() { }


}
