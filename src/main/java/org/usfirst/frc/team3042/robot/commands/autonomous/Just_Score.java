package org.usfirst.frc.team3042.robot.commands.autonomous;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Arm_SetPosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Autonomous Mode (Default) ******************************************************
 * This is a basic autonomous routine */
public class Just_Score extends SequentialCommandGroup {

  public Just_Score(double rotationGoal, double extensionGoal) {
    SequentialCommandGroup score = new SequentialCommandGroup(new InstantCommand(() -> Robot.gripper.extend()), new Wait(1), new Arm_SetPosition(RobotMap.kArmDrivePosition, RobotMap.kExtendDrivePosition));

    addCommands(new InstantCommand(() -> Robot.gripper.retract()));
    addCommands(new Wait(1));

    addCommands(new Arm_SetPosition_Auto(rotationGoal, extensionGoal));
    addCommands(score);
  }
}