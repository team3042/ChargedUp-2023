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
	public static final int kBackLeftDriveMotorPort = 15;
    public static final int kBackLeftTurningMotorPort = 2;
	public static final int kBackRightDriveMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 5;
	public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
	public static final int kFrontRightDriveAbsoluteEncoderPort = 9;
	public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
	public static final int kBackRightDriveAbsoluteEncoderPort = 12;
	public static final int kRotationMotorPort = 13;
	public static final int kExtendMotorPort = 14;

	/** Drivetrain Settings ***************************************************/
	public static final double kP_X_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards. Only used for autonomous path-following
    public static final double kP_Y_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards. Only used for autonomous path-following	
    public static final double kP_THETA_CONTROLLER = 9.6421; // TODO: Find this value by characterizing the drivetrain with SysID, and then by using guess & check afterwards. Only used for autonomous path-following
	
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
	public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = -87.4; // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
	public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = 54.5; // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kBackLeftDriveEncoderReversed = false;
	public static final boolean kBackLeftTurningEncoderReversed = false;
	public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = 153.77; //155 // More negative turns wheel more to the left (counter-clockwise)
    
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
	public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = 107.0; // More negative turns wheel more to the left (counter-clockwise)

	
	public static final double JOYSTICK_DRIVE_SCALE = 0.7; // Determines the max driving speed of the robot
	public static final double JOYSTICK_DRIVE_SCALE_LOW = 0.2; // Determines driving speed of the robot when in slow mode

	/** Swerve Module Settings ************************************************/
	public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Convert wheel diameter in inches to meters
	public static final double kDriveMotorGearRatio = 1 / 6.75; // Gear Ratio of the Drive Motor
	public static final double kTurnMotorGearRatio = 1 / 12.8; // Gear Ratio of the Turning Motor
	public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Convert rotations to meters
	public static final double kTurningEncoderRot2Rad = kTurnMotorGearRatio * 2 * Math.PI; // Convert rotations to radians
	public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; // Convert RPM to meters/second
	public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60; // Convert RPM to radians/sec
	public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(14); // Determines the maximum driving speed of the robot
	public static final double kPhysicalMaxTurningSpeedRadiansPerSecond = Math.PI * 3; // Determines the maximum turning speed of the robot
	public static final double kP_Turning = 0.4;
	public static final double nominalVoltage = 12.0;
    public static final int driveCurrentLimit = 30; // used to be 80
    public static final int steerCurrentLimit = 20;

	/** Arm Subsystem Settings */
	public static final boolean rotationMotorReversed = false;
    public static final boolean extendMotorReversed = false;
	public static final double rotationThreshold = 50;
	public static final double extensionThreshold = 80;
	public static final double rotation_kP = 0.008;
	public static final double extension_kP = 0.0003;
	public static final int maxArmLength = 15200; // Measured in encoder counts
	public static final double shoulderCountsPerDegree = 11.666667;
	public static final double armStartingDegrees = 7.5; // measured in degrees
	public static final double levelVoltageRetracted = 0.3115; // TODO: Adjust this if needed 
	public static final double levelVoltageExtended = 0.5; // TODO: Adjust this if needed  

	/** Arm/Extension Positions */
	// Intake from the floor // 
	public static final double kIntakeArmPosition = 190; // measured in encoder counts
	public static final double kIntakeExtendPosition = 0.45; // measured in % of extension
	// Middle goal //
	public static final double kMidArmPosCone = 870; // measured in encoder counts
	public static final double kScoringExtendPosition1 = 0.42; // measured in % of extension
	// Upper goal
	public static final double kHighArmPosCone = 1020; // measured in encoder counts
	public static final double kScoringExtendPosition2 = 0.95; // measured in % of extension
	// Intake from the shelf
	public static final double kShelfIntakeArmPosition = 880; // measured in encoder counts
	public static final double kShelfIntakeExtendPosition = 0.20; // measured in % of extension
	// Driving around
	public static final double kArmDrivePosition = 50; // measured in encoder counts
	public static final double kExtendDrivePosition = 0.02; // measured in % of extension

	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.07;
	public static final double ANGLE_TOLERANCE = 2.0;
	
	/** USB ports *************************************************************/					
	public static final int USB_JOYSTICK_LEFT 	= 0;
	public static final int USB_JOYSTICK_RIGHT 	= 1;
	public static final int USB_GAMEPAD 		= 2;
	
	/** PCM (Pneumatics Control Module) Channels ******************************/
	public static final int Gripper_PistonID1 = 0;
	public static final int Gripper_PistonID2 = 1;

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
	public static final Log.Level   LOG_LIMELIGHT 						= Log.Level.DEBUG;
 
	// Controller Input Axes //
    public static final int DRIVER_XBOX_USB_PORT = 0; // USB port that the controller is plugged in to
	public static final int GUNNER_XBOX_USB_PORT = 1; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 4;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 3;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 3;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_AXIS = 7;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int PREV_BUTTON = 7;
    public static final int START_BUTTON = 8;

	
}