// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance_On_Station extends CommandBase {

  Drivetrain drivetrain = Robot.drivetrain;
  public boolean overshot = false;
  
  /** Creates a new Balance_On_Station command. */
  public Balance_On_Station() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When we get onto the charging station, slow down but continue driving forward

		// System.out.println(moving_fast);
    
  if (drivetrain.pitchAngle() <= -1){

      drivetrain.drive(-0.3, 0, 0, false); // Move forward fast using the drive() method
      
    } else if (drivetrain.pitchAngle() >= 1){
      
      overshot = true;
      drivetrain.drive(0.3, 0, 0, false); // Move forward fast using the drive() method
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules(); // Stop the drivetrain by calling stopModules()
    (new Drivetrain_XStance()).schedule(); // Schedule a new Drivetrain_XStance command (runs the command)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If we are moving slowly and our pitch drops below a certain number of degrees, then we are done!
    return  /*overshot &&*/ drivetrain.pitchAngle() >= -1 && drivetrain.pitchAngle() <= 1;
  }
}