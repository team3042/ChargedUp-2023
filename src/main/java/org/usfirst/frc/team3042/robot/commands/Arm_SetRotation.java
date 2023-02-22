// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

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


    // When going to the intake position, we will wait for the extension to move first before rotating the arm
    if (rotationPositionGoal != RobotMap.kIntakeArmPosition) {

      // THIS BLOCK OF CODE BELOW ROTATES THE ARM SHOULDER //
      // double minimalVoltage = RobotMap.levelVoltage * Math.cos(arm.getArmAngle() * (arm.getExtendMotorPosition()/RobotMap.maxArmLength));
      arm.setVoltageRotationMotor(rotationError); //+minimal voltage, * kP rotation

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