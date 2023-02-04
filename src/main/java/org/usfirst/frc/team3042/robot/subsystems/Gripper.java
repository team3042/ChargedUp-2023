// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  DoubleSolenoid gripperPiston;
  boolean isExtended;

  /** Creates a new gripper. */
  public Gripper() {
    gripperPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.Gripper_PistonID1, RobotMap.Gripper_PistonID2);
    isExtended = true;
  }

  public void extend() {
    gripperPiston.set(Value.kForward);
    isExtended = true;
  }

  public void retract() {
    gripperPiston.set(Value.kReverse);
    isExtended = false;
  }

  public void toggle() {
    if(!isExtended){
      extend();
    }
    else if (isExtended){
      retract();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
