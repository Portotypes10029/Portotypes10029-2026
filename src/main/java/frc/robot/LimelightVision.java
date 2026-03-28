package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVision {

    public final NetworkTable frontLimelight;
    public final NetworkTable turretLimelight;

    public LimelightVision() {
        frontLimelight = NetworkTableInstance.getDefault().getTable("frontLimelight");
        turretLimelight = NetworkTableInstance.getDefault().getTable("turretLimelight");
    }

    public boolean hasTarget(NetworkTable limelight) {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public int getTagID(NetworkTable limelight) {
        return (int) limelight.getEntry("tid").getDouble(-1);
    }

    public boolean seesTag(int tagID, NetworkTable limelight) {
        return hasTarget(limelight) && getTagID(limelight) == tagID;
    }

    public double getTX(NetworkTable limelight) {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getTY(NetworkTable limelight) {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getTA(NetworkTable limelight) {
        return limelight.getEntry("ta").getDouble(0);
    }
}