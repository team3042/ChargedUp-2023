package org.usfirst.frc.team3042.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

/** Drivetrain Gyro Turn ******************************************************
 * Command for turning in place to a set angle. */
public class Drivetrain_GyroTurn extends CommandBase {
	/** Configuration Constants ***********************************************/
	private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double kP = RobotMap.kP_GYRO;
	private static final double ANGLE_TOLERANCE = RobotMap.ANGLE_TOLERANCE;
	
	/** Instance Variables ****************************************************/
	Drivetrain drivetrain = Robot.drivetrain;
	Log log = new Log(LOG_LEVEL, SendableRegistry.getName(drivetrain));
	double goalAngle, error;
	
	/** Drivetrain Gyro Turn ************************************************** 
	 * Required subsystems will cancel commands when this command is run.
	 * distance is given in physical units matching the wheel diameter unit
	 * speed is given in physical units per second. The physical units should 
	 * match that of the Wheel diameter.
	 * @param angle (degrees) */
	public Drivetrain_GyroTurn(double angle) {
		log.add("Constructor", Log.Level.TRACE);
		goalAngle = angle;
		addRequirements(drivetrain);
	}
	
	/** initialize ************************************************************
	 * Called just before this Command runs the first time */
	public void initialize() {
		log.add("Initialize", Log.Level.TRACE);
		drivetrain.stopModules();
		drivetrain.zeroGyro();
	}

	/** execute ***************************************************************
	 * Called repeatedly when this Command is scheduled to run */
	public void execute() {
		error = goalAngle - drivetrain.getGyroAngle();
		
		double correction = kP * error; // correction will be measured in radians per second
		
		// Set a maximum rotation speed
		correction = Math.min(1, correction);
		correction = Math.max(-1, correction);
	
		drivetrain.drive(0, 0, -1 * correction, false);		
	}
	
	/** isFinished ************************************************************	
	 * Make this return true when this Command no longer needs to run execute() */
	public boolean isFinished() {
		return Math.abs(error) < ANGLE_TOLERANCE; // End this command when our current heading is within the tolerance
	}

	// Called once the command ends or is interrupted.
	public void end(boolean interrupted) {
		log.add("End", Log.Level.TRACE);
		drivetrain.stopModules(); // Stop all modules when this command ends
	}
}