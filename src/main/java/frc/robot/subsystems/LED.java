/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem controls the LEDS. All it does is take in a
 * number for each pattern and it will spit that number onto the usb serial port
 * for the arduino to handle
 */
public class LED extends Subsystem implements ISubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LED instance = null;

  public static LED getInstance() {
    instance = new LED();
    SmartDashboard.putNumber("Inc", Timer.getFPGATimestamp());
    return instance;
  }

  private SerialPort LEDPort;

  private String[] patternNames = { "Error", "Rainbow Gradient", "Rainbow Chase", "Color Fill", "Blink",
      "Moving Gradient", "ColorChase" };

  private int currentPattern = 0;

  public LED() {
    try {
      LEDPort = new SerialPort(9600, SerialPort.Port.kUSB);
    } catch (Exception e) {
    }
  }

  public void setRainbowGradient(int delay) {
    try {
      if (currentPattern != 1 && instance!=null)
        LEDPort.writeString("1" + "," + delay);
      currentPattern = 1;
    } catch (Exception e) {
    }
  }

  public void setRainbowChase(int delay) {
    try {
      if (currentPattern != 2 && instance!=null)
        LEDPort.writeString("2" + "," + delay);
      currentPattern = 2;
    } catch (Exception e) {
    }
  }

  public void setColorFill(int HSVcolor, int delay) {
    try {
      if (currentPattern != 3 && instance!=null)
        LEDPort.writeString("3" + "," + HSVcolor + "," + delay);
      currentPattern = 3;
    } catch (Exception e) {
    }
  }

  public void setBlink(int HSVcolor, int delay) {
    try {
      if (currentPattern != 4 && instance!=null)
        LEDPort.writeString("4" + "," + HSVcolor + "," + delay);
      currentPattern = 4;
    } catch (Exception e) {
    }
  }

  public void setMovingGradient(int HSVcolor1, int HSVcolor2, int delay) {
    try {
      if (currentPattern != 5 && instance!=null)
        LEDPort.writeString("5" + "," + HSVcolor1 + "," + HSVcolor2 + "," + delay);
      currentPattern = 5;
    } catch (Exception e) {
    }
  }

  public void setColorChase(int HSVcolor1, int HSVcolor2, int delay) {
    try {
      if (currentPattern != 6 && instance!=null)
        LEDPort.writeString("6" + "," + HSVcolor1 + "," + HSVcolor2 + "," + delay);
      currentPattern = 6;
    } catch (Exception e) {
    }
  }

  @Override
  public void outputSmartdashboard() {
    SmartDashboard.putString("LED Pattern", patternNames[currentPattern] + " " + currentPattern);
  }

  @Override
  public void zeroSensors() {

  }

  @Override
  public void resetSubsystem() {
    LEDPort.reset();
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
