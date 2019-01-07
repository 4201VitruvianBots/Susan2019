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
        return limelightTable.getEntry("camMode").getNumber( 0).intValue();
    }

    public void setCameraMode(int camMode) {
        limelightTable.getEntry("camMode").setNumber(camMode);
    }

    public void initDefaultCommand() { }


}
