// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Balance_On_Station extends InstantCommand {

  Drivetrain drivetrain = Robot.drivetrain;

  private double error;
  private double currentAngle;
  private double drivePower;

  private static final Log.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN;
	private static final double kP = RobotMap.kP_GYRO;
  
  public Balance_On_Station() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = Robot.drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    
  }


  @Override
  public void execute() {

    this.currentAngle = drivetrain.getGyroAngle();

    error = 0 - currentAngle;
  }
}
