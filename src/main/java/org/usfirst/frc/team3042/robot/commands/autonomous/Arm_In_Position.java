// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm_In_Position extends CommandBase {

  double rotationGoal;
  double extensionGoal;
  boolean extensionGoalReached;
  boolean rotationGoalReached;
  Arm arm = Robot.arm;
  /** Creates a new Arm_In_Position. */
  public Arm_In_Position(double rGoal, double eGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    rotationGoal = rGoal;
    extensionGoal = eGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    extensionGoalReached = false;
    rotationGoalReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    extensionGoalReached = (Math.abs(extensionGoal - arm.getExtendMotorPosition()) <= RobotMap.extensionThreshold);
    rotationGoalReached = (Math.abs(rotationGoal - arm.getRotationMotorPosition()) <= RobotMap.rotationThreshold);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extensionGoalReached && rotationGoalReached;
  }
}
