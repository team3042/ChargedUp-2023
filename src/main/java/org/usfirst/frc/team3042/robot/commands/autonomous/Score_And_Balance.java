package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Arm_SetPosition;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_GyroStraight;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is a basic autonomous routine */
public class Score_And_Balance extends SequentialCommandGroup {

  public Score_And_Balance(double rotationGoal, double extensionGoal) {
    SequentialCommandGroup score = new SequentialCommandGroup (new InstantCommand(() -> Robot.gripper.extend()), new Wait(1), new Arm_SetPosition_Auto(rotationGoal, RobotMap.kExtendDrivePosition));
    ParallelCommandGroup balance = new ParallelCommandGroup( new Drive_Balance(), new Arm_SetPosition(RobotMap.kArmDrivePosition, RobotMap.kExtendDrivePosition));
    SequentialCommandGroup scoreAndBalance = new SequentialCommandGroup(score, balance);

    addCommands(new InstantCommand(() -> Robot.gripper.retract()));
    addCommands(new Wait(1));

    addCommands(new Arm_SetPosition_Auto(rotationGoal, extensionGoal));
    addCommands(scoreAndBalance);

  }
}