// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax rotationMotor;
  private final CANSparkMax extendMotor;

  /** Creates a new Arm Subsystem */
  public Arm() {
    rotationMotor = new CANSparkMax(RobotMap.kArmRotationMotor, MotorType.kBrushless);
    extendMotor = new CANSparkMax(RobotMap.kArmExtendMotor, MotorType.kBrushless);

    // Configure Motor Settings

    rotationMotor.restoreFactoryDefaults();
    extendMotor.restoreFactoryDefaults();

    rotationMotor.setInverted(RobotMap.rotationMotorReversed);
    extendMotor.setInverted(RobotMap.extendMotorReversed);

    rotationMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setIdleMode(IdleMode.kBrake);
  }

  // Methods for setting power to the motor
  public void setPowerRotationMotor(double percentPower) {
    rotationMotor.set(percentPower);
  }
  public void setPowerExtendMotor(double percentPower) {
    extendMotor.set(percentPower);
  }

  // Methods for stopping the motors
  public void stopRotationMotor() {
    setPowerRotationMotor(0);
  }
  public void stopExtendMotor() {
    setPowerExtendMotor(0);
  }

  public void setPosition(double armPosition, double extensionPosition) {
    double armError = armPosition - getRotationMotorPosition();
    double extensionError = extensionPosition - getExtendMotorPosition();

    while (Math.abs(armError) > RobotMap.armThreshold || Math.abs(extensionError) > RobotMap.extensionThreshold) {
      setPowerRotationMotor(armError * RobotMap.arm_kP);
      setPowerExtendMotor(extensionError * RobotMap.extension_kP);
    }
  }

  // Encoder methods for getting the motor position
  public double getRotationMotorPosition() {
    return rotationMotor.getEncoder().getPosition();
  }
  public double getExtendMotorPosition() {
    return extendMotor.getEncoder().getPosition();
  }
  // Encoder methods for getting the motor velocity
  public double getRotationMotorVelocity() {
    return rotationMotor.getEncoder().getVelocity();
  }
  public double getExtendMotorVelocity() {
    return extendMotor.getEncoder().getVelocity();
  }

  // Reset both encoders to 0
  public void resetEncoders() {
    rotationMotor.getEncoder().setPosition(0);
    extendMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}