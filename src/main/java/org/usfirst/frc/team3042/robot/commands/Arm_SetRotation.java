// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE: This command sets ONLY the rotation position

public class Arm_SetRotation extends CommandBase {
  Arm arm = Robot.arm;

  public double rotationPositionGoal;

  /** Creates a new Arm_SetPosition command. */
  public Arm_SetRotation(double rotationGoal) { // rotationGoal is measured in encoder counts
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.rotationPositionGoal = rotationGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationError = arm.getRotationMotorPosition() - rotationPositionGoal;

    // THIS BLOCK OF CODE BELOW ROTATES THE ARM SHOULDER //
    double minimalVoltage = RobotMap.levelVoltage * Math.sin(arm.getArmAngle()); // TODO: Also scale this minimalVoltage by how far the arm is extended somehow?
    arm.setVoltageRotationMotor(minimalVoltage + (rotationError * RobotMap.rotation_kP)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors if the command is interrupted
    arm.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // We never want this command to end
  }
}