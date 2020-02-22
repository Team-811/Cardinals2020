/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.awt.Color;
import java.awt.color.*;
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

    public void initialize() {
        pixy.init();
        pixy.setLamp((byte) 0, (byte) 0);
    }

    public Vision() {
        initialize();
    }

    private boolean lampStatus = false;

    /**
     * Toggles the two bright white lights on the Pixy2
     */
    public void toggleLamp() {
        lampStatus = !lampStatus;

        if (lampStatus) {
            pixy.setLamp((byte) 1, (byte) 0);
            pixy.setLED(currentLEDColor);
        } else {
            pixy.setLamp((byte) 0, (byte) 0);
            pixy.setLED(currentLEDColor);
        }
    }

    private java.awt.Color currentLEDColor = Color.black;

    /**
     * Set the color of the Pixy2 LED
     * 
     * @param color RGB Value in Hex
     */
    public void setLEDColor(java.awt.Color color) {
        currentLEDColor = color;
        pixy.setLED(Color.BLACK);
    }

    @Override
    public void outputSmartdashboard() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void resetSubsystem() {
        initialize();
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
