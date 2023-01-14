package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;
import org.usfirst.frc.team3042.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.GenericHID;

public static final GenericHID controller = new GenericHID(RobotMap.CONTROLLER_USB_PORT_ID) ; // Instantiate our controller at the specified USB port 