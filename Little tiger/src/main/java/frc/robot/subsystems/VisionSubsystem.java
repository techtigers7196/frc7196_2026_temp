package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {
 private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
 private NetworkTableEntry tx = table.getEntry("tx");
 private NetworkTableEntry ty = table.getEntry("ty");
 private NetworkTableEntry ta = table.getEntry("ta");
 private NetworkTableEntry tv = table.getEntry("tv");
 private NetworkTableEntry tid = table.getEntry("tid");

public double[] getXYA()
{
 double x = tx.getDouble(0.0);
 double y  = ty.getDouble(0.0);
 double a = ta.getDouble(0.0);
 return new double[]{x, y, a};
}
}
