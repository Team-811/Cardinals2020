/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.lib.Pixy2.Pixy2;
import frc.robot.lib.Pixy2.links.SPILink;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem deals with vision tracking with the Pixy2 Camera
 */

public class Vision extends Subsystem implements ISubsystem {

    private static Vision instance = new Vision();

    public static Vision getInstance() {
        return instance;
    }

    private static final Pixy2 pixy = Pixy2.createInstance(new SPILink());;

    public static void initialize() {
        pixy.init();
        pixy.setLamp((byte) 0, (byte) 1);
    }

    public Vision() {

    }

    private boolean color;

    public void toggleLed() {
        color = !color;
        if (color)
            pixy.setLED(255, 0, 0);
        else
            pixy.setLED(0, 255, 0);
    }

    @Override
    public void outputSmartdashboard() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void resetSubsystem() {

    }

    @Override
    public void testSubsystem() {

    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());

    }
}
