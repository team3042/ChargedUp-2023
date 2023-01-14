package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** OI ************************************************************************
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot. */
public class OI { 
   
    /** Declare Instance Variables ****************************************************/
	Log log = new Log(RobotMap.LOG_OI, "OI");
    public static final GenericHID controller = new GenericHID(RobotMap.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port 
    public static final Joystick joyLeft = new Joystick(RobotMap.USB_JOYSTICK_LEFT); // Instantiate our left joystick at the specified USB port 
    public static final Joystick joyRight = new Joystick(RobotMap.USB_JOYSTICK_RIGHT); // Instantiate our right joystick at the specified USB port 

	/** OI ********************************************************************
	 * Assign commands to the buttons and triggers*/
	public OI() {
		log.add("OI Constructor", Log.Level.TRACE);

        // Bind commands to buttons on the controller/joysticks here! //
		new Trigger(() -> joyLeft.getTrigger()).onTrue(new InstantCommand(Robot.drivetrain::zeroGyro, Robot.drivetrain)); // Zero the gyro, this is helpful at the start of a match for field-oriented driving
		new Trigger(() -> joyRight.getTrigger()).onTrue(new Drivetrain_XStance()); // Defensive X-stance command

        // Example using the A button on the controller below:
        // new Trigger(() -> controller.getRawButton(RobotMap.A_BUTTON)).onTrue(new Drivetrain_XStance());
	}

    /** Access to the driving axes values *****************************
	 * A negative can be added to make pushing forward positive/negative. */
	public double getXSpeed() {
		double joystickValue = joyRight.getY();
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}
	public double getYSpeed() {
		double joystickValue = joyRight.getX();
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}
	public double getZSpeed() {
		double joystickValue = joyLeft.getX();
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxTurningSpeedRadiansPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}	
 
    // The rest of the class
    // blah
}