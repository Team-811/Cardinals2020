/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.DatagramPacket;

import java.util.Arrays;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */
public class Vision extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Port used for receiving variables
  private final int PORT = 5800;

  // Network temporary variables
  private DatagramSocket socket;
  private boolean running;
  private byte[] buf = new byte[28];
  private String[] data = new String[4];

  // Final data
  private int targetId = -1;
  private double offsetX = -1;
  private double distance = -1;
  private double angle = -1;

  public Vision()
  {
    try {
      // port 5800-5810
      socket = new DatagramSocket(PORT);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void onLoop() {
    // receive information
    DatagramPacket packet = new DatagramPacket(buf, buf.length);
    try {
      socket.receive(packet);
    } catch (Exception e) {
      e.printStackTrace();
    }

    // put information into String[]
    data = new String(packet.getData(), 0, packet.getLength()).split(",");
    
    // parse all data from String[] into variables
    targetId =   Integer.parseInt(data[0]);
    offsetX  = Double.parseDouble(data[1]);
    distance = Double.parseDouble(data[2]);
    angle    = Double.parseDouble(data[3]);
  }

  public int getTargetId() {
    return targetId;
  }

  public double getOffsetX() {
    return offsetX;
  }

  public double getDistance() {
    return distance;
  }

  public double getAngle() {
    return angle;
  }

  @Override
  public void outputSmartdashboard() 
  {
    
  }

  @Override
  public void zeroSensors() 
  {
    
  }

  @Override
  public void resetSubsystem() 
  {
    
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
