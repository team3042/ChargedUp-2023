package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;

import edu.wpi.first.math.util.Units;

/** RobotMap ******************************************************************
 * This is the robot configuration file. */
public class RobotMap {	
	/** Robot Size Parameters **************************************************/
	public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Distance between centers of right and left wheels on robot (in meters)
    public static final double WHEEL_BASE = Units.inchesToMeters(23.5); // Distance between centers of front and back wheels on robot (in meters)

	/** CAN ID numbers ********************************************************/
	public static final int kFrontLeftDriveMotorPort = 3;
	public static final int kFrontLeftTurningMotorPort = 4;
	public static final int kFrontRightDriveMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 7;
	public static final int kBackLeftDriveMotorPort = 1;
    public static final int kBackLeftTurningMotorPort = 2;
	public static final int kBackRightDriveMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 5;
	public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
	public static final int kFrontRightDriveAbsoluteEncoderPort = 9;//11
	public static final int kBackLeftDriveAbsoluteEncoderPort = 11;//9
	public static final int kBackRightDriveAbsoluteEncoderPort = 12;

	/** Drivetrain Settings ***************************************************/
	public static final double kP_X_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards
    public static final double kP_Y_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards	
    public static final double kP_THETA_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards
	//swerve part :3
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
	public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = 49.55; // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
	public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = 183.48; // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kBackLeftDriveEncoderReversed = false;
	public static final boolean kBackLeftTurningEncoderReversed = false;
	public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = -35.49; // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
	public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = 182.81; // More negative turns wheel more to the left (counter-clockwise)

	/** Swerve Module Settings ************************************************/
	public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Convert wheel diameter in inches to meters
	public static final double kDriveMotorGearRatio = 1 / 6.75; // Gear Ratio of the Drive Motor
	public static final double kTurnMotorGearRatio = 1 / 12.8; // Gear Ratio of the Turning Motor
	public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Convert rotations to meters
	public static final double kTurningEncoderRot2Rad = kTurnMotorGearRatio * 2 * Math.PI; // Convert rotations to radians
	public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; // Convert RPM to meters/second
	public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60; // Convert RPM to radians/sec
	public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14); // Convert max speed from feet/sec to meters/sec
	public static final double kPhysicalMaxTurningSpeedRadiansPerSecond = Math.PI * 3; // TODO: This can be tuned to your liking
	public static final double kP_Turning = 0.4; // TODO: We need to tune this value through testing! (this could cause X stance to perform poorly?)
	public static final double nominalVoltage = 12.0;
    public static final int driveCurrentLimit = 80;
    public static final int steerCurrentLimit = 20;
	
	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.005;
	public static final double ANGLE_TOLERANCE = 2.0;
	
	/** USB ports *************************************************************/					
	public static final int USB_JOYSTICK_LEFT 	= 0;
	public static final int USB_JOYSTICK_RIGHT 	= 1;
	public static final int USB_GAMEPAD 		= 2;

	/** Logger Settings *******************************************************/
	public static final String 		LOG_FILE_FORMAT 					= "yyyy-MM-dd-hhmmss";
	public static final String 		LOG_TIME_FORMAT 					= "hh:mm:ss:SSS";
	public static final String 		LOG_DIRECTORY_PATH 					= "/home/lvuser/logs/";
	public static final String 		LOG_TIME_ZONE 						= "America/Chicago";
	public static final boolean 	LOG_TO_CONSOLE 						= true;
	public static final boolean 	LOG_TO_FILE 						= false;
	public static final Log.Level 	LOG_GLOBAL 							= Log.Level.DEBUG;
	public static final Log.Level 	LOG_ROBOT 							= Log.Level.TRACE;
	public static final Log.Level	LOG_OI 								= Log.Level.TRACE;
	public static final Log.Level	LOG_AXIS_TRIGGER 					= Log.Level.ERROR;
	public static final Log.Level	LOG_POV_BUTTON						= Log.Level.ERROR;
	
	/** Subsystem Log Levels **************************************************/
	public static final Log.Level	LOG_DRIVETRAIN						= Log.Level.TRACE;
 
	// Controller Input Axes //
    public static final int CONTROLLER_USB_PORT_ID = 2; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 3;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 2;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 1;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_AXIS = 7;
    public static final int RIGHT_TRIGGER_AXIS = 8;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;
}