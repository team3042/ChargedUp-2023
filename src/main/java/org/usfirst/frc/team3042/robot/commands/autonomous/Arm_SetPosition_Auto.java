// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm_SetPosition_Auto extends CommandBase {
  Arm arm = Robot.arm;

  public double rotationPositionGoal;
  public double extensionPositionGoal;
  boolean extensionGoalReached;
  boolean rotationGoalReached;
  public double rotationError;
  double extensionPower;

  /** Creates a new Arm_SetPosition_Auto command. */
  public Arm_SetPosition_Auto(double rotationGoal, double extensionPercent) { // rotationGoal is measured in encoder counts
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    this.rotationPositionGoal = rotationGoal;
    this.extensionPositionGoal = extensionPercent * RobotMap.maxArmLength; // convert from a percent to actual encoder counts
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
    rotationError = rotationPositionGoal - arm.getRotationMotorPosition();
    double extensionError = extensionPositionGoal - arm.getExtendMotorPosition();

    extensionGoalReached = (Math.abs(extensionError) <= RobotMap.extensionThreshold);
    rotationGoalReached = (Math.abs(rotationError) <= RobotMap.rotationThreshold);
    // Rotation threshold determines how far from the drive position we should be to wait for the arm before moving the extension
    boolean inDrivePosition = Math.abs(arm.getRotationMotorPosition() - RobotMap.kArmDrivePosition) <= RobotMap.rotationThreshold;

      // THIS BLOCK OF CODE BELOW ROTATES THE ARM SHOULDER //
      double minimalVoltage =(RobotMap.levelVoltageRetracted * (1 - (arm.getExtendMotorPosition()/RobotMap.maxArmLength)) + RobotMap.levelVoltageExtended * (arm.getExtendMotorPosition()/RobotMap.maxArmLength));
      double rotationVoltage = minimalVoltage + (rotationError * RobotMap.rotation_kP);
      rotationVoltage = Math.min(6, rotationVoltage);
      rotationVoltage = Math.max(-2, rotationVoltage);
      arm.setVoltageRotationMotor(rotationVoltage); 
    
    // We are only going to move the extension if the arm is NOT in the intake position
    if (!inDrivePosition) {

      // THIS BLOCK OF CODE BELOW MOVES THE EXTENSION //
      if (!extensionGoalReached) {
        extensionPower = extensionError * RobotMap.extension_kP;
        extensionPower = Math.min(0.8, extensionPower);
        extensionPower = Math.max(-0.8, extensionPower);
        
        extensionPower = Math.copySign(Math.max(Math.abs(extensionPower), 0.2), extensionError);

        arm.setPowertoExtend(extensionPower);
      } else {
        arm.stopExtendMotor();
      }

    } else {
      arm.stopExtendMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors if the command is interrupted
    arm.stopExtendMotor();
    double minimalVoltage =(RobotMap.levelVoltageRetracted * (1 - (arm.getExtendMotorPosition()/RobotMap.maxArmLength)) + RobotMap.levelVoltageExtended * (arm.getExtendMotorPosition()/RobotMap.maxArmLength));
    arm.setVoltageRotationMotor(minimalVoltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extensionGoalReached && rotationGoalReached; // It's going to end now
  }
} 