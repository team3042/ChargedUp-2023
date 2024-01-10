package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** Autonomous Mode (Default) ******************************************************
 * This is a basic autonomous routine */
public class Drive_Out_And_Balance extends SequentialCommandGroup {

  public Drive_Out_And_Balance() {

    CommandBase initialDrive = new Drivetrain_GyroStraight(.8, -2, 0);
    CommandBase slowerDrive = new Drivetrain_GyroStraight(0.7, -1.5, 0);
    CommandBase backupDrive = new Drivetrain_GyroStraight(.8, 2, 0);

    SequentialCommandGroup driveOut = new SequentialCommandGroup(initialDrive, slowerDrive, backupDrive, new Balance_On_Station());
    // SequentialCommandGroup balance = new SequentialCommandGroup(new Drivetrain_GyroStraight(0.75, 1, 0), new Balance_On_Station());

    addCommands(driveOut);

  }
}