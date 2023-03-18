package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Limelight *****************************************************************
 * Subsystem for the Limelight camera */
public class Limelight extends SubsystemBase {
	/** Configuration Constants ***********************************************/
  	private static final Log.Level LOG_LEVEL = RobotMap.LOG_LIMELIGHT;
	
	/** Instance Variables ****************************************************/
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(this));
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); // Data is sent over NetworkTables

	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	NetworkTableEntry tv = table.getEntry("tv");  

	public NetworkTableEntry pipeline = table.getEntry("pipeline");
	public NetworkTableEntry led = table.getEntry("ledMode");
	
	/** Limelight ************************************************************/
	public Limelight() {
		log.add("Constructor", LOG_LEVEL);
		pipeline.setNumber(0); // Set the default vision pipeline
	}
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem. */
	public void initDefaultCommand() {
		setDefaultCommand(null); 
	}

	// LED Controls //
	public void LEDs_On() {
		table.getEntry("ledMode").setNumber(3);
	}
	public void LEDs_Off() {
		table.getEntry("ledMode").setNumber(1);
	}
	
	// Generic Targetting Methods //
	public double returnHorizontalAngleError() { // Horizontal ANGLE of error
		double x = tx.getDouble(0.0);
		return x;
	}
	public double returnVerticalAngleError() { // Vertical ANGLE of error
		double y = ty.getDouble(0.0);
		return y;
	}
	public double returnTargetArea() { // Whether the Limelight has any valid targets: 0 (false) or 1 (true)
		double area = ta.getDouble(0.0);
		return area;
	}
	public double returnValidTarget() { // AREA of the target between 0% and 100% of frame
		double target = tv.getDouble(0.0);
		return target;
	}

	// Fancy Apriltag Methods //
	public double[] ApriltagPose() { // Get the 3D pose of the nearest Apriltag
		return table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
	}
	public double getApriltagX() {
		return ApriltagPose()[0];
	}
	public double getApriltagY() {
		return ApriltagPose()[1];
	}
	public double getApriltagZ() {
		return ApriltagPose()[2];
	}
	public double getApriltagRoll() {
		return ApriltagPose()[3];
	}
	public double getApriltagPitch() {
		return ApriltagPose()[4];
	}
	public double getApriltagYaw() {
		return ApriltagPose()[5];
	}
}