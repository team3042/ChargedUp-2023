// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm_SetPosition extends CommandBase {
  Arm arm = Robot.arm;

  public double rotationPosition;
  public double extensionPosition;

  /** Creates a new Arm_SetPosition command. */
  public Arm_SetPosition(double rotationGoal, double extensionPercent) { // rotationGoal is measured in encoder counts
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.rotationPosition = rotationGoal;
    this.extensionPosition = extensionPercent * RobotMap.maxArmLength; // convert from a percent to actual encoder counts
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotationError = arm.getRotationMotorPosition() - rotationPosition;
    double minimalVoltage = RobotMap.levelVoltage * Math.cos(arm.getArmAngle() * (arm.getExtendMotorPosition()/RobotMap.maxArmLength));
    arm.setVoltageRotationMotor(minimalVoltage + (rotationError * RobotMap.rotation_kP)); 

    double extensionError = arm.getExtendMotorPosition() - extensionPosition;
    if (Math.abs(extensionError) > RobotMap.extensionThreshold){
      arm.setPowertoExtend(Math.copySign(0.2, extensionError)); // increase percent power to make arm move faster
      // if it keeps oscillating because it can't reach the correct position, use (extensionError * kP) instead
    } else {
      arm.stopExtendMotor();
    }
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