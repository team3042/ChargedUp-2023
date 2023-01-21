// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.OI;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
  Drivetrain drivetrain = Robot.drivetrain;
  OI oi = Robot.oi;
  /** Creates a new DriveCommand. */
  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ySpeed = oi.getYSpeed();
		double xSpeed = oi.getXSpeed();
		double zSpeed = oi.getZSpeed();

		drivetrain.drive(xSpeed, ySpeed, zSpeed, true);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
