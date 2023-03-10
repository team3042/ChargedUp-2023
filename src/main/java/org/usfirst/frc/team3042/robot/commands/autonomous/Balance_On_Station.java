// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Balance_On_Station extends CommandBase {

  Drivetrain drivetrain = Robot.drivetrain;
  boolean moving_fast = true;

  /** Creates a new Balance_On_Station command. */
  public Balance_On_Station() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moving_fast = true; // We start the command by moving fast until we get onto the charging station
    drivetrain.drive(-2, 0, 0, false); // Move forward fast using the drive() method
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When we get onto the charging station, slow down but continue driving forward
    if(moving_fast == true && drivetrain.pitchAngle() >= 15) {
      moving_fast = false;
      drivetrain.drive(-0.5,0,0,false); // Drive forward slowly with the drive() method
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
    return !moving_fast && drivetrain.pitchAngle() <= 10;
  }
}