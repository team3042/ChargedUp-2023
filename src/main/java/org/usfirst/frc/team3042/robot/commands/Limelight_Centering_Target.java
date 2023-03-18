package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Limelight_Centering_Target extends CommandBase {
    Limelight limelight = Robot.limelight;
  
    /** Creates a new Arm_SetPosition command. */
    public Limelight_Centering_Target() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(limelight); // cancel all other arm commands when this runs
    }


    @Override 
    public void initialize() {

        double error = limelight.returnHorizontalError();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() { 
        
        return true;
    }



}