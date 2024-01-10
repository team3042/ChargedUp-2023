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
public class Score_And_Exit_Long extends SequentialCommandGroup {

  public Score_And_Exit_Long(double rotationGoal, double extensionGoal) {
    ParallelCommandGroup leaveZone = new ParallelCommandGroup(new Drivetrain_GyroStraight(1.5, -0.9, 0), new Arm_SetPosition(RobotMap.kArmDrivePosition, RobotMap.kExtendDrivePosition));
    SequentialCommandGroup score = new SequentialCommandGroup (new InstantCommand(() -> Robot.gripper.extend()), new Wait(1));
    SequentialCommandGroup scoreAndLeave = new SequentialCommandGroup(score, leaveZone);

    addCommands(new InstantCommand(() -> Robot.gripper.retract()));
    addCommands(new Wait(1));

    addCommands(new Arm_SetPosition_Auto(rotationGoal, extensionGoal));
    addCommands(scoreAndLeave);

  }
}