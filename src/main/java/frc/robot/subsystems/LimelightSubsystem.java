package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem {

    public double getTX(String limelightName) {
        return getTable(limelightName).getEntry("tx").getDouble(0.0);
    }

    public double getTY(String limelightName) {
        return getTable(limelightName).getEntry("ty").getDouble(0.0);
    }

    public double getTA(String limelightName) {
        return getTable(limelightName).getEntry("ta").getDouble(0.0);
    }

    public double getID(String limelightName) {
        return getTable(limelightName).getEntry("tid").getDouble(-1);
    }

    public boolean hasTarget(String limelightName) {
        return getTable(limelightName).getEntry("tv").getDouble(0) == 1;
    }

    private NetworkTable getTable(String name) {
        return NetworkTableInstance.getDefault().getTable(name);
    }
}