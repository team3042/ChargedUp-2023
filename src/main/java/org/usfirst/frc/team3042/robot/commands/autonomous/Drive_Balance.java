package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is a basic autonomous routine */
public class Drive_Balance extends SequentialCommandGroup {

  public Drive_Balance() {

    SequentialCommandGroup balance = new SequentialCommandGroup(new Drivetrain_GyroStraight(0.75, -1, 0),new Wait(1), new Balance_On_Station());

    addCommands(balance);



  }
}