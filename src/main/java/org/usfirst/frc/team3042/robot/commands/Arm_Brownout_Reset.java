// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE: This command sets ONLY the extension position

public class Arm_Brownout_Reset extends CommandBase {
  Arm arm = Robot.arm;


  /** Creates a new Arm_SetPosition command. */
  public Arm_Brownout_Reset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm); // cancel all other arm commands when this runs

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(arm.ExtensionLimitSwitch.get()) {

      arm.setPowertoExtend(-0.35);
    }else{

      arm.setPowertoExtend(0);
    }

    if(!arm.ExtensionLimitSwitch.get() && arm.RotationLimitSwitch.get()) {

      arm.setVoltageRotationMotor(-1 + (RobotMap.levelVoltage * Math.sin(arm.getArmAngle())));
    }else{
      arm.setPowerToRotation(0);
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

      return (!arm.ExtensionLimitSwitch.get() && !arm.RotationLimitSwitch.get());
  }
}