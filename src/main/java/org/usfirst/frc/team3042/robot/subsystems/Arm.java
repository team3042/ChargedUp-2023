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
    rotationMotor = new CANSparkMax(RobotMap.kRotationMotorPort, MotorType.kBrushless);
    extendMotor = new CANSparkMax(RobotMap.kExtendMotorPort, MotorType.kBrushless);

    // Configure Motor Settings

    rotationMotor.restoreFactoryDefaults();
    extendMotor.restoreFactoryDefaults();

    rotationMotor.setInverted(RobotMap.rotationMotorReversed);
    extendMotor.setInverted(RobotMap.extendMotorReversed);

    rotationMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setIdleMode(IdleMode.kBrake);
  }

  // Methods for setting power to the motors
  public void setPowerToRotation(double percentPower) {
    rotationMotor.set(percentPower);
  }
  public void setPowertoExtend(double percentPower) {
    extendMotor.set(percentPower);
  }

  // Methods for setting voltage to the motors
  public void setVoltageRotationMotor(double volts) {
    volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
    volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

    rotationMotor.setVoltage(volts);
  }
  public void setVoltageExtendMotor(double volts) {
    volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
    volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

    extendMotor.setVoltage(volts);
  }

  // Methods for stopping the motors
  public void stopRotationMotor() {
    setPowerToRotation(0);
  }
  public void stopExtendMotor() {
    setPowertoExtend(0);
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

  public double getArmAngle(){

    double encoderCounts = getRotationMotorPosition();

    return encoderCounts * RobotMap.shoulderCountsPerDegree + RobotMap.armStartingDegrees;

  } //RETURNS DEGREES




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}