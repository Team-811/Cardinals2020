/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.awt.Color;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.Pixy2.Pixy2;
import frc.robot.lib.Pixy2.Pixy2CCC;
import frc.robot.lib.Pixy2.Pixy2CCC.Block;
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
        pixyC = pixy.getCCC();
    }

    public Vision() {

    }

    Pixy2CCC pixyC;

    /**
     * Stores retrieved blocks from Pixy2
     */
    ArrayList<Block> blocks = new ArrayList<Block>();

    /**
     * 
     * @return True if one or more balls is in the frame
     */
    public boolean ballDetected() {
        detectBalls();

        if (!blocks.isEmpty()) {
            return true;
        }

        return false;
    }

    /**
     * 
     * @return Number of balls in frame
     */
    public int numBallsDetected() {
        detectBalls();
        return blocks.size();
    }

    /**
     * Get which block is recognized better by the camera to track it
     * 
     * @return Largest target in the frame
     */
    private Block getBiggestBlock() {
        // Gets the number of "blocks", identified targets, that match signature 1 on
        // the Pixy2,
        // does not wait for new data if none is available,
        // and limits the number of returned blocks to 25, for a slight increase in
        // efficiency
        int blockCount = pixy.getCCC().getBlocks(true, 1, 10);
        System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
        if (blockCount <= 0) {
            return null; // If blocks were not found, stop processing
        }
        ArrayList<Block> blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
        Block largestBlock = null;
        for (Block block : blocks) { // Loops through all blocks and finds the widest one
            if (largestBlock == null) {
                largestBlock = block;
            } else if (block.getWidth() > largestBlock.getWidth()) {
                largestBlock = block;
            }
        }
        return largestBlock;
    }

    /**
     * Method to get new pixy blocks before doing measurements
     */
    private void detectBalls() {
        pixyC.getBlocks(true, 1, 10);
        blocks = pixyC.getBlocks();
    }

    /**
     * Attempt to align the robot toward the largest target (ball). If the intake is on,
     * this will also attempt to grab it
     */
    public void alignLargestTarget() {
        Block target = getBiggestBlock();
        double blockX = target.getX();
        double centered = 150;
        if (blockX >= centered)
        {//Right
            while (blockX >= centered)
            {
                target = getBiggestBlock();
                blockX = target.getX();
                Robot.drivetrain.driveWithJoy(.3, 0, .5);
            }
            Robot.drivetrain.stopDrivetrain();
        }
        else
        {//left
            while (blockX <= centered)
            {
                target = getBiggestBlock();
                blockX = target.getX();
                Robot.drivetrain.driveWithJoy(.3, 0, -.5);
            }
            Robot.drivetrain.stopDrivetrain();
        }
    }

    private boolean lampStatus = false;

    /**
     * Toggles the two bright white lights on the Pixy2
     */
    public void toggleLamp() {
        lampStatus = !lampStatus;

        if (lampStatus) {
            pixy.setLamp((byte) 1, (byte) 0);
            setLEDColor(currentLEDColor);
        } else {
            pixy.setLamp((byte) 0, (byte) 0);
            setLEDColor(currentLEDColor);
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
        pixy.setLED(currentLEDColor);
    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putString("LED Color", currentLEDColor.toString());
        SmartDashboard.putBoolean("Lamp Status", lampStatus);
        SmartDashboard.putBoolean("Ball Detected", ballDetected());
        SmartDashboard.putNumber("# Of Balls", numBallsDetected());
        SmartDashboard.putNumber("Height", pixy.getFrameHeight());
        SmartDashboard.putNumber("Width", pixy.getFrameWidth());
        try {
            SmartDashboard.putNumber("Ball X", blocks.get(0).getX());
            SmartDashboard.putNumber("Ball Y", blocks.get(0).getY());
        } catch (Exception e) {
            SmartDashboard.putNumber("Ball X", -1);
            SmartDashboard.putNumber("Ball Y", -1);
        }

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
