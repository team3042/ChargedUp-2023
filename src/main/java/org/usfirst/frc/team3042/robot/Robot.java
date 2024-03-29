package org.usfirst.frc.team3042.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.DriveCommand;
import org.usfirst.frc.team3042.robot.commands.autonomous.AutonomousMode_Default;
import org.usfirst.frc.team3042.robot.commands.autonomous.Drive_Balance;
import org.usfirst.frc.team3042.robot.commands.autonomous.Score_And_Balance;
import org.usfirst.frc.team3042.robot.commands.autonomous.Score_Drive_Out_And_Balance;
import org.usfirst.frc.team3042.robot.commands.autonomous.Drive_Out_And_Balance;
import org.usfirst.frc.team3042.robot.commands.autonomous.Score_And_Exit_Long;
import org.usfirst.frc.team3042.robot.commands.autonomous.Score_And_Exit_Short;
import org.usfirst.frc.team3042.robot.commands.autonomous.Just_Score;
import org.usfirst.frc.team3042.robot.subsystems.Arm;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Gripper;
import org.usfirst.frc.team3042.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera; // Uncomment if you want to use a USB webcam
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;


/** Robot *********************************************************************
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource directory. */
public class Robot extends TimedRobot { 

	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_ROBOT;
	private Log log = new Log(LOG_LEVEL, "Robot");

	int counter = 0;

	/** Create Subsystems *****************************************************/
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gripper gripper = new Gripper();
	public static final Arm arm = new Arm();
	public static final Limelight limelight = new Limelight();

	public static final PowerDistribution pdh = new PowerDistribution();

	public static final OI oi = new OI(); // ALL SUBSYSTEMS MUST BE INSTANTIATED BEFORE THIS LINE!
	
	CommandBase autonomousCommand;
	SendableChooser<CommandBase> chooser = new SendableChooser<CommandBase>();

	UsbCamera camera1; // Uncomment if you want to use a USB webcam

	/** robotInit *************************************************************
	 * This function is run when the robot is first started up and should be used for any initialization code. */
	public void robotInit() {
		log.add("Robot Init", Log.Level.TRACE);
		drivetrain.setDefaultCommand(new DriveCommand());
		
		drivetrain.zeroGyro();
		arm.resetEncoders();

		// Autonomous Routines //
		chooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
		chooser.addOption("Just Score", new Just_Score(RobotMap.kMidArmPosCone,RobotMap.kScoringExtendPosition1));
		chooser.addOption("Score Mid + Drive Out Short Side", new Score_And_Exit_Short(RobotMap.kMidArmPosCone,RobotMap.kScoringExtendPosition1));
		chooser.addOption("Score Mid + Drive Out Long Side", new Score_And_Exit_Long(RobotMap.kMidArmPosCone,RobotMap.kScoringExtendPosition1));
		chooser.addOption("Score Mid + Balance", new Score_And_Balance(RobotMap.kMidArmPosCone,RobotMap.kScoringExtendPosition1));
		chooser.addOption("Drive Out Of Community + Balance", new Drive_Out_And_Balance());
		chooser.addOption("Score + Drive Out Of Community + Balance", new Score_Drive_Out_And_Balance(RobotMap.kMidArmPosCone, RobotMap.kScoringExtendPosition1));
		chooser.addOption("Balance", new Drive_Balance());
		SmartDashboard.putData("Auto Mode", chooser);

		// Start up the webcam and configure its resolution and framerate

		 camera1 = CameraServer.startAutomaticCapture(0);
		 camera1.setResolution(320, 240);
		 camera1.setFPS(15);

	}

	/** disabledInit **********************************************************
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the robot is disabled. */
	public void disabledInit() {
		log.add("Disabled Init", Log.Level.TRACE);
	}

	/** disabledPeriodic ******************************************************
	 * Called repeatedly while the robot is in disabled mode. */
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();

		SmartDashboard.putString("BackLeft State", drivetrain.getBackLeft().getState().toString());
		SmartDashboard.putString("FrontLeft State", drivetrain.getFrontLeft().getState().toString());
		SmartDashboard.putString("BackRight State", drivetrain.getBackRight().getState().toString());
		SmartDashboard.putString("FrontRight State", drivetrain.getFrontRight().getState().toString());

		
		SmartDashboard.putNumber("Extension Encoder counts", arm.getExtendMotorPosition());
		SmartDashboard.putNumber("Rotation Encoder Counts", arm.getRotationMotorPosition());

		if(counter >= 100) {
			SmartDashboard.putNumber("FrontRight AbsEncoder", drivetrain.getFrontRight().getAbsoluteEncoderRadians() * 180/Math.PI);
			SmartDashboard.putNumber("FrontLeft AbsEncoder", drivetrain.getFrontLeft().getAbsoluteEncoderRadians() * 180/Math.PI);
			SmartDashboard.putNumber("BackRight AbsEncoder", drivetrain.getBackRight().getAbsoluteEncoderRadians() * 180/Math.PI);
			SmartDashboard.putNumber("BackLeft AbsEncoder", drivetrain.getBackLeft().getAbsoluteEncoderRadians() * 180/Math.PI);
			counter = 0;
		}

		// Check if limit switches are pressed
		if (!arm.ExtensionLimitSwitch.get()){ 
			arm.resetExtendEncoder();
		}
		if (!arm.RotationLimitSwitch.get()){ 
			arm.resetRotationEncoder();
		}
		
		counter++;
	}

	/** autonomousInit ********************************************************
	 * Runs once at the start of autonomous mode. */
	public void autonomousInit(){
		log.add("Autonomous Init", Log.Level.TRACE);
		
		limelight.pipeline.setNumber(0);

		autonomousCommand = chooser.getSelected();
		
		// schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}


	}

	/** autonomousPeriodic ****************************************************
	 * This function is called periodically during autonomous */
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();

		SmartDashboard.putNumber("Extension Encoder counts", arm.getExtendMotorPosition());
		SmartDashboard.putNumber("Rotation Encoder Counts", arm.getRotationMotorPosition());

		
		       
        SmartDashboard.putString("BackLeft State", drivetrain.getBackLeft().getState().toString());
		SmartDashboard.putString("FrontLeft State", drivetrain.getFrontLeft().getState().toString());
		SmartDashboard.putString("BackRight State", drivetrain.getBackRight().getState().toString());
		SmartDashboard.putString("FrontRight State", drivetrain.getFrontRight().getState().toString());

		// Check if limit switches are pressed
		if (!arm.ExtensionLimitSwitch.get()){ 
			arm.resetExtendEncoder();
		}
		if (!arm.RotationLimitSwitch.get()){ 
			arm.resetRotationEncoder();
		}


	}
	
	/** teleopInit ************************************************************
	 * This function is called when first entering teleop mode. */
	public void teleopInit() {

		
		log.add("Teleop Init", Log.Level.TRACE);
		
		limelight.pipeline.setNumber(0);

		// This makes sure that the autonomous command stops running when teleop starts. 
		//If you want the autonomous command to continue until interrupted by another command, remove this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}  
	}

	/** teleopPeriodic ********************************************************
	 * This function is called periodically during operator control */
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		       
        // SmartDashboard.putString("BackLeft State", drivetrain.getBackLeft().getState().toString());
		// SmartDashboard.putString("FrontLeft State", drivetrain.getFrontLeft().getState().toString());
		// SmartDashboard.putString("BackRight State", drivetrain.getBackRight().getState().toString());
		// SmartDashboard.putString("FrontRight State", drivetrain.getFrontRight().getState().toString());
		SmartDashboard.putNumber("Extension Encoder counts", arm.getExtendMotorPosition());
		SmartDashboard.putNumber("Rotation Encoder Counts", arm.getRotationMotorPosition());
		SmartDashboard.putBoolean("limit esxtension", arm.ExtensionLimitSwitch.get());
		SmartDashboard.putNumber("gyroscope", drivetrain.getRawGyroAngle());
		SmartDashboard.putNumber("Extension Amerage", pdh.getCurrent(5));

		// You can uncomment the line below if you need to tune levelVoltage:
		// arm.setVoltageRotationMotor(RobotMap.levelVoltageExtended);

		// Manual Control of the arm motors (leave these lines commented out unless you need them):
		// Robot.arm.setPowerToRotation(OI.gunnerController.getRawAxis(RobotMap.LEFT_VERTICAL_JOYSTICK_AXIS)); // Multiply by -1 to invert joystick
		// Robot.arm.setPowertoExtend(-1 * OI.gunnerController.getRawAxis(RobotMap.RIGHT_VERTICAL_JOYSTICK_AXIS)); // Multiply by -1 to invert joystick

		// Check if limit switches are pressed
		if (!arm.ExtensionLimitSwitch.get()){ 
			arm.resetExtendEncoder();
		}
		if (!arm.RotationLimitSwitch.get()){ 
			arm.resetRotationEncoder();
		}

		if (pdh.getCurrent(5) > RobotMap.coneAmp){

			arm.stopExtendMotor();
		}
	} 

	// Give this method the name of a .json autonomous path and it will return a PPSwerveControllerCommand for that path :)
	public static SequentialCommandGroup constructTrajectoryCommand(String pathName, double velocityMax, double accelMax) {
		
		PathPlannerTrajectory path = PathPlanner.loadPath(pathName, velocityMax, accelMax); 

		// Add kinematics to ensure max speed is actually obeyed
		PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(path, drivetrain::getPose, drivetrain.getkDriveKinematics(),

		// Position contollers
		new PIDController(RobotMap.kP_X_CONTROLLER, 0, 0),
		new PIDController(RobotMap.kP_Y_CONTROLLER, 0, 0),
		new PIDController(RobotMap.kP_THETA_CONTROLLER, 0, 0),

		drivetrain::setModuleStates, drivetrain);

		PathPlannerState initialState = (PathPlannerState)path.sample(0); // Define the initial state of the trajectory

		return new SequentialCommandGroup(
			new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation))),
			swerveControllerCommand,
			new InstantCommand(() -> drivetrain.stopModules())
		);
	}
}