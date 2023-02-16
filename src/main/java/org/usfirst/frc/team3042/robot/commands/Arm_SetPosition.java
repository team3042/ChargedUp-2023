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
  public double extentionPosition;

  /** Creates a new Arm_SetPosition command. */
  public Arm_SetPosition(double rotationPosition, double extensionPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    rotationPosition = this.rotationPosition;
    extensionPosition = this.extentionPosition;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    extentionPosition = extentionPosition * RobotMap.maxArmLength; //extention position is a percent
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // FIXME: This execute() method needs to be completed as described below!

    double rotationError = arm.getRotationMotorPosition() - rotationPosition;
    double minimalVoltage = RobotMap.levelVoltage * Math.cos(arm.getArmAngle() * (arm.getExtendMotorPosition()/RobotMap.maxArmLength));
    arm.setVoltageRotationMotor(minimalVoltage + (rotationError * RobotMap.rotation_kP)); 
      // NOTE: The minimal voltage depends on the angle of the arm though, this is tricky!
      // NOTE: You will need to calculate the minimal voltage needed for when the arm is level
      //       Then, the minimal voltage needed at any given time will simply be levelVoltage * cosine(angle of the arm)
      //       You will need to determine the angle of the arm based on the position of the rotation motor

    
    double extentionError = arm.getExtendMotorPosition() - extentionPosition;
    // TODO: If the error is within the threshold, stop the extension motor
    if (Math.abs(extentionError) > RobotMap.extensionThreshold){
      arm.setPowertoExtend(Math.copySign(0.2, extentionError)); //increase percent power to make arm more faster
      // if it keeps oscillating because it can't reach the correct position, add power * kP
    }

      // NOTE: The extension shouldn't have to fight against gravity, so we can just stop the motor when in position
    // TODO: Otherwise, set power to the extension motor equal to the error * kP
    // Like this: arm.setPowerExtendMotor(error * RobotMap.extension_kP);
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