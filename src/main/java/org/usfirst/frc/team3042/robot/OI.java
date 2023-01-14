package org.usfirst.frc.team3042.robot;

import edu.wpi.first.wpilibj.GenericHID;


   

/** OI ************************************************************************
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot. */
public class OI { 
   
    // some declarations
     public static final GenericHID controller = new GenericHID(RobotMap.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port 

    public double getYSpeed() {
        return 0;
    }

    public double getXSpeed() {
        return 0;
    }

    public double getZSpeed() {
        return 0;
    }
 
    // The rest of the class
    // blah
}
 