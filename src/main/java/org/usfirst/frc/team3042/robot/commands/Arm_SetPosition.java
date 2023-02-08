// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm_SetPosition extends CommandBase {
  Arm arm = Robot.arm;

  /** Creates a new Arm_SetPosition command. */
  public Arm_SetPosition(double rotationPosition, double extensionPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // FIXME: This execute() method needs to be completed as described below!
    // TODO: Calculate the rotation position error
    // TODO: If the error is within the threshold, set some minimal power to the rotation motor to counteract gravity
      // TODO: Would this minimal power depend on the current angle of the arm though? This seems tricky!
    // TODO: Otherwise, set power to the rotation motor equal to the error * kP

    // TODO: Calculate the extension position error
    // TODO: If the error is within the threshold, stop the extension motor
    // TODO: Otherwise, et power to the extension motor equal to the error * kP
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors if the command is interrupted
    arm.stopExtendMotor();
    arm.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // We never want this command to end
  }
}