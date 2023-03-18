package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;
import org.usfirst.frc.team3042.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

// Assists the driver by aligning with and driving towards the nearest Apriltag
public class Limelight_Centering_Target extends CommandBase {
    Limelight limelight = Robot.limelight;
    Drivetrain drivetrain = Robot.drivetrain;

    double angleError;
    double xDistanceError;
    double yDistanceError;

    double angleTolerance = 2; // TODO: This will NEED to be tuned (should be somewhat small)
    double xDistanceTolerance = 0.2; // TODO: This will NEED to be tuned (should be somewhat small)
    double yDistanceTolerance = 2.0; // TODO: This will NEED to be tuned (this will affect how close you drive to the Apriltag)

    double kP_Drive = 0.5; // TODO: This will NEED to be tuned
  
    /** Creates a new Limelight_Centering_Target command. */
    public Limelight_Centering_Target() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limelight);
      addRequirements(drivetrain);
    }

    @Override 
    public void initialize() {
        limelight.LEDs_On();
    }

    @Override
    public void execute() {
        angleError = limelight.getApriltagYaw();
        xDistanceError = limelight.getApriltagX();
        yDistanceError = limelight.getApriltagY();

        drivetrain.drive(yDistanceError*kP_Drive, xDistanceError*kP_Drive, angleError*RobotMap.kP_GYRO, false); // Yes I know it's weird, but X and Y are flipped for our drive method
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
        limelight.LEDs_Off();
    }

    @Override
    public boolean isFinished() { 
        return Math.abs(angleError) <= angleTolerance && Math.abs(xDistanceError) <= xDistanceError && Math.abs(yDistanceError) <= yDistanceError;
    }
}