/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.lib.vision.VisionList;
import frc.robot.lib.vision.VisionTarget;
import frc.robot.Constants;
import frc.robot.RobotState;

import java.net.DatagramSocket;
import java.util.ArrayList;
import java.util.List;
import java.net.DatagramPacket;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */
public class Vision extends Subsystem implements ISubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Vision instance = new Vision();

  public static Vision getInstance() {
    return instance;
  }



  // Port used for receiving variables
  private final int PORT = 5800;

  // Network temporary variables
  private DatagramSocket socket;
  private byte[] buf = new byte[28];
  private String[] data = new String[4];

  private VisionList targets = new VisionList();
  private RobotState robot_state_ = RobotState.getInstance();



  public Vision() {
    try {
      // port 5800-5810
      socket = new DatagramSocket(PORT);
    } catch (Exception e) {
      System.out.println("Can't initialize vision socket");
    }
  }

  public void onLoop() {
    // receive information
    DatagramPacket packet = new DatagramPacket(buf, buf.length);
    try {
      socket.receive(packet);
    } catch (Exception e) {
      //Ignore
    }
    // put information into String[]
    data = new String(packet.getData(), 0, packet.getLength()).split(",");

    targets.clearList(); //Clear previous targets before adding more
    
    // parse all data from String[] into variables
    for(int i = 0; i < data.length; i+= 3)
    {
      double xOffset = Double.parseDouble(data[i]);
      double distance = Double.parseDouble(data[i + 1]);
      double angle = Double.parseDouble(data[i + 2]);

      targets.addTarget(new VisionTarget(xOffset, distance, angle));
    }

    targets.setTimestamp(Timer.getFPGATimestamp() - Constants.cameraLatency); //Set timestamp of targets

    robot_state_.addVisionUpdate(targets.getTimestamp(), targets); //Adds vision targets to robot state class to be used for targeting

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
