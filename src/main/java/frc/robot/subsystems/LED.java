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

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem controls the LEDS. All it does is take in a
 * number for each pattern and it will spit that number onto the usb serial port
 * for the arduino to handle
 */
public class LED extends Subsystem implements ISubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LED instance = new LED();

  public static LED getInstance() {
    return instance;
  }

  private SerialPort LEDPort;

  // set default LED pattern at start-up
  private Integer defaultPattern = 3;

  private String[] patternNames = { "ERROR", "Rainbow Dot Trail", "Rainbow", "Rainbow Gradient", "Rainbow Snake",
      "Red/Yellow Snake", "Blue/Yellow Snake", "Confetti", "America", "Red Flame", "Blue Flame", "BG Flame",
      "Rainbow Flame", "Rainbow Chase", "Red", "Green" };

  private int currentPattern = 0;

  public LED() {
    try {
      LEDPort = new SerialPort(9600, SerialPort.Port.kUSB);
      LEDPort.writeString(defaultPattern.toString());
      currentPattern = defaultPattern;
    } catch (Exception e) {

    }

  }

  /**
   * Set the LEDs to a given preset
   * 
   * @param preset Number of the preset. See the string array in this subsystem to
   *               determine which number represents which pattern
   */
  public void setLEDs(int preset) {
    try {
      switch (preset) {
      case 0:
        currentPattern = 0;
        break;
      // RainbowDotTrail
      case 1:
        LEDPort.writeString("1");
        currentPattern = 1;
        break;
      // Rainbow
      case 2:
        LEDPort.writeString("2");
        currentPattern = 2;
        break;
      // RainbowGradient
      case 3:
        LEDPort.writeString("3");
        currentPattern = 3;
        break;
      // RainbowSnake
      case 4:
        LEDPort.writeString("4");
        currentPattern = 4;
        break;
      // RedYellowSnake
      case 5:
        LEDPort.writeString("5");
        currentPattern = 5;
        break;
      // BlueYellowSnake
      case 6:
        LEDPort.writeString("6");
        currentPattern = 6;
        break;
      // Confetti
      case 7:
        LEDPort.writeString("7");
        currentPattern = 7;
        break;
      // America
      case 8:
        LEDPort.writeString("8");
        currentPattern = 8;
        break;
      // RedFlame
      case 9:
        LEDPort.writeString("9");
        currentPattern = 9;
        break;
      // BlueFlame
      case 10:
        LEDPort.writeString("10");
        currentPattern = 10;
        break;
      // BGFlame
      case 11:
        LEDPort.writeString("11");
        currentPattern = 11;
        break;
      // RainbowFlame
      case 12:
        LEDPort.writeString("12");
        currentPattern = 12;
        break;
      // ChaseRainbow
      case 13:
        LEDPort.writeString("13");
        currentPattern = 13;
        break;
      // Red
      case 14:
        LEDPort.writeString("14");
        currentPattern = 14;
        break;
      // Green
      case 15:
        LEDPort.writeString("15");
        currentPattern = 15;
        break;
      }
    } catch (Exception e) {
      currentPattern = 0;
    }
  }

  @Override
  public void outputSmartdashboard() {
    SmartDashboard.putString("LED Pattern", patternNames[currentPattern]);
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
