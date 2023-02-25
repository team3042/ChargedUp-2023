// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE: This command sets ONLY the extension position

public class Arm_SetExtend extends CommandBase {
  Arm arm = Robot.arm;

  public double extensionPositionGoal;
  boolean extensionGoalReached = false; // Determines whent he command should end

  /** Creates a new Arm_SetPosition command. */
  public Arm_SetExtend(double extensionPercent) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm); // cancel all other arm commands when this runs

    this.extensionPositionGoal = extensionPercent * RobotMap.maxArmLength; // convert from a percent to actual encoder counts
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extensionGoalReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extensionError = extensionPositionGoal - arm.getExtendMotorPosition();

    extensionGoalReached = (Math.abs(extensionError) <= RobotMap.extensionThreshold);

    // THIS BLOCK OF CODE BELOW MOVES THE EXTENSION //
    if (!extensionGoalReached) {
      arm.setPowertoExtend(Math.copySign(0.4, extensionError)); // Increase the percent power if you want to make the extension move faster
       // At some point we will want to use PID control instead: (extensionError * extension_kP) instead of (Math.copySign(0.2, extensionError))
    } else {
      arm.stopExtendMotor();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors if the command is interrupted
    arm.stopExtendMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extensionGoalReached; // End the command when our extensionGoal has been reached
  }
}