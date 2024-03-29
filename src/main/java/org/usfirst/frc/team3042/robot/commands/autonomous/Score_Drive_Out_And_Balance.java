package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Arm_SetPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is a basic autonomous routine */
public class Score_Drive_Out_And_Balance extends SequentialCommandGroup {

  public Score_Drive_Out_And_Balance(double rotationGoal, double extensionGoal) {
    SequentialCommandGroup score = new SequentialCommandGroup (new InstantCommand(() -> Robot.gripper.extend()), new Wait(0.5), new Arm_SetPosition_Auto(rotationGoal, RobotMap.kExtendDrivePosition));
    ParallelCommandGroup balance = new ParallelCommandGroup( new Drive_Out_And_Balance(), new Arm_SetPosition(RobotMap.kArmDrivePosition, RobotMap.kExtendDrivePosition));
    SequentialCommandGroup scoreAndBalance = new SequentialCommandGroup(score, balance);

    addCommands(new InstantCommand(() -> Robot.gripper.retract()));
    addCommands(new Wait(0.5));

    addCommands(new Arm_SetPosition_Auto(rotationGoal, extensionGoal));
    addCommands(scoreAndBalance);

  }
}