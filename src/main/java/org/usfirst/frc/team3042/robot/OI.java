package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Log;
import org.usfirst.frc.team3042.robot.commands.Drivetrain_XStance;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
	//public static final XboxController driverController = new XboxController(RobotMap.DRIVER_XBOX_USB_PORT); // TODO: Uncomment this to use an Xbox controller for the driver
	//public static final XboxController gunnerController = new XboxController(RobotMap.GUNNER_XBOX_USB_PORT); // TODO: Uncomment this to use an Xbox controller for the gunner

	/** OI ********************************************************************
	 * Assign commands to the buttons and triggers */
	public OI() {
		log.add("OI Constructor", Log.Level.TRACE);

        // Bind commands to buttons on the controllers/joysticks below! //
		new Trigger(() -> joyLeft.getTrigger()).onTrue(new InstantCommand(Robot.drivetrain::zeroGyro, Robot.drivetrain)); // Zero the gyro, this is helpful at the start of a match for field-oriented driving
		new Trigger(() -> joyRight.getTrigger()).onTrue(new Drivetrain_XStance()); // Defensive X-stance command
		//new Trigger(() -> getLeftTrigger(driverController)).onTrue(new InstantCommand(Robot.drivetrain::zeroGyro, Robot.drivetrain)); // TODO: Uncomment this to use an Xbox controller for the driver
		//new Trigger(() -> getRightTrigger(driverController)).onTrue(new Drivetrain_XStance()); // TODO: Uncomment this to use an Xbox controller for the driver

        // Example using the A button on a generic logitech controller:
        // new Trigger(() -> controller.getRawButton(RobotMap.A_BUTTON)).onTrue(new Drivetrain_XStance());

		// Example using the X button on a Xbox controller:
        // new Trigger(() -> driverController.getXButton()).onTrue(new Drivetrain_XStance());
	}

    /** Access to the driving axes values *****************************
	 * A negative can be added to make pushing forward positive/negative. */
	public double getXSpeed() {
		double joystickValue = joyRight.getY(); // TODO: Delete this and use the line below instead if you want to use an Xbox controller for the driver
		//double joystickValue = driverController.getRightY(); // TODO: Uncomment this to use an Xbox controller for the driver
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}
	public double getYSpeed() {
		double joystickValue = joyRight.getX(); // TODO: Delete this and use the line below instead if you want to use an Xbox controller for the driver
		//double joystickValue = driverController.getRightX(); // TODO: Uncomment this to use an Xbox controller for the driver
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxSpeedMetersPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}
	public double getZSpeed() {
		double joystickValue = joyLeft.getX(); // TODO: Delete this and use the line below instead if you want to use an Xbox controller for the driver
		//double joystickValue = driverController.getLeftX(); // TODO: Uncomment this to use an Xbox controller for the driver
		if (Math.abs(joystickValue) < 0.01) { // This is our deadband
			return 0.0;
		}
		else {
			return joystickValue * RobotMap.kPhysicalMaxTurningSpeedRadiansPerSecond * -1; // Multiply by -1 reverses the direction
		}	
	}	

	// These methods can be used to detect when the left/right trigger of a XboxController is pressed (convert the analog trigger axis to a boolean value: pressed or not pressed)
	public boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() >= 0.95;
	  }
	public boolean getRightTrigger(XboxController controller) {
		return controller.getRightTriggerAxis() >= 0.95;
	}
 
    // The rest of the class
    // blah
}