package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.OI;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain X Stance **************************************************
 * Command for defensive X Stance */
public class Drivetrain_XStance extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));

	SwerveModuleState FrontLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
	SwerveModuleState FrontRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
	SwerveModuleState BackLeftState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
	SwerveModuleState BackRightState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
	
	/** Drivetrain X Stance **********************************************
	 * Required subsystems will cancel commands when this command is run */
	public Drivetrain_XStance() {
		log.add("Constructor", Log.Level.TRACE);
		addRequirements(drivetrain);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		drivetrain.getFrontLeft().setDesiredState(FrontLeftState, true);
		drivetrain.getFrontRight().setDesiredState(FrontRightState, true);
		drivetrain.getBackLeft().setDesiredState(BackLeftState, true);
		drivetrain.getBackRight().setDesiredState(BackRightState, true);
	}

	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return Math.abs(OI.driverController.getRawAxis(0)) > 0.05 || Math.abs(OI.driverController.getRawAxis(1)) > 0.05 || Math.abs(OI.driverController.getRawAxis(2)) > 0.05;
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		drivetrain.stopModules();
	}
}