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

  /** Creates a new Arm_SetPosition command. */
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
    double rotationError = rotationPositionGoal - arm.getRotationMotorPosition();
    double extensionError = extensionPositionGoal - arm.getExtendMotorPosition();

    extensionGoalReached = (Math.abs(extensionError) <= RobotMap.extensionThreshold);
    rotationGoalReached = (Math.abs(rotationError) <= RobotMap.rotationThreshold);
    // Rotation threshold determines how far from the drive position we should be to wait for the arm before moving the extension
    boolean inDrivePosition = Math.abs(arm.getRotationMotorPosition() - RobotMap.kArmDrivePosition) <= RobotMap.rotationThreshold;

    // When going to the intake position, we will wait for the extension to move first before rotating the arm
    if (rotationPositionGoal != RobotMap.kArmDrivePosition || (rotationPositionGoal == RobotMap.kArmDrivePosition && extensionGoalReached)) {

      // THIS BLOCK OF CODE BELOW ROTATES THE ARM SHOULDER //
      double minimalVoltage = RobotMap.levelVoltage * Math.sin(arm.getArmAngle()) * (RobotMap.levelVoltageRetracted * (1 - ((arm.getExtendMotorPosition()/RobotMap.maxArmLength) * 100)) + RobotMap.levelVoltageExtended * 1 * ((arm.getExtendMotorPosition()/RobotMap.maxArmLength) * 100)); // We might need to also scale this based on how far extended the arm is
      arm.setVoltageRotationMotor(minimalVoltage + (rotationError * RobotMap.rotation_kP)); 

    } else {
      arm.stopRotationMotor();;
    }
    
    // We are only going to move the extension if the arm is NOT in the intake position
    if (!inDrivePosition) {

      // THIS BLOCK OF CODE BELOW MOVES THE EXTENSION //
      if (!extensionGoalReached) {
        arm.setPowertoExtend(Math.copySign(0.4, extensionError)); // Increase the percent power if you want to make the extension move faster
        // At some point we will want to use PID control instead: (extensionError * extension_kP) instead of (Math.copySign(0.2, extensionError))
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
    arm.setVoltageRotationMotor(RobotMap.levelVoltage * Math.sin(arm.getArmAngle()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extensionGoalReached || rotationGoalReached; // It's going to end now
  }
}