/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Utility.TimerCommand;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem deals with the color wheel and contains methods
 * to automatically perform both rotation and position control
 */

public class ColorWheel extends Subsystem implements ISubsystem {

    private static ColorWheel instance = new ColorWheel();

    public static ColorWheel getInstance() {
        return instance;
    }

    // Store the possible color wheel values in a color matcher
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public ColorWheel() {
        cWheelMotor = new CANSparkMax(RobotMap.WHEEL, MotorType.kBrushless);
        cSensor = new ColorSensorV3(I2C.Port.kOnboard);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    private CANSparkMax cWheelMotor;
    private ColorSensorV3 cSensor;

    private int count = 0;

    /**
     * Rotate the color wheel 4 times. Blink LEDs green when complete
     * 
     * @param speed (0-1)
     */
    public void rotationControl(double speed) {
        ColorMatchResult initial = m_colorMatcher.matchClosestColor(cSensor.getColor());
        double start = Timer.getFPGATimestamp();
        ColorMatchResult current = m_colorMatcher.matchClosestColor(cSensor.getColor());
        ColorMatchResult prev = current;

        // the color sensor will see the same color 8 times during one rotation
        if (count < 8) {
            cWheelMotor.set(speed);
            current = m_colorMatcher.matchClosestColor(cSensor.getColor());
            // SmartDashboard.putNumber("count", count);
            SmartDashboard.putBoolean("Current=initial", current.color == initial.color);
            SmartDashboard.putBoolean("Current=previous", current.color != prev.color);

            if (current.color == initial.color && current.color != prev.color) {
                count++;
                prev = current;
            }

        }
        stopColorWheel();
        // Robot.led.setBlink(96, 200);
    }

    public void rotationControlManual(double speed) {

        String color = getCurrentColor();
        if (speed == 0) {
            Robot.setDefaultLED();
            cWheelMotor.set(0);
        } else {
            cWheelMotor.set(speed);
        }
        SmartDashboard.putString("Color2", color);
    }

    public void rotationControlTimed(double time, double speed) {
        double initial = Timer.getFPGATimestamp();

        while (Timer.getFPGATimestamp() - initial < time) {
            cWheelMotor.set(speed);
        }

        stopColorWheel();
    }

    /**
     * Spins the color wheel to the given color, Blinks LEDs green when complete
     * 
     * @param speed (0-1)
     */
    public void positionControl(double speed) {
        // String target = Robot.ds.getGameSpecificMessage();
        String target = "R";

        if (target.length() > 0) {
            switch (target.charAt(0)) {
            case 'B':
                goToColor(kBlueTarget, speed);
                break;
            case 'G':
                goToColor(kGreenTarget, speed);
                break;
            case 'R':
                goToColor(kRedTarget, speed);
                break;
            case 'Y':
                goToColor(kYellowTarget, speed);
                break;
            default:
                break;
            }
        } else {
            // do nothing if no color has been specified
        }
    }

    /**
     * Go to a specified color on the color wheel
     * 
     * @param c
     * @param speed
     */
    private void goToColor(Color c, double speed) {

        double start = Timer.getFPGATimestamp();

        while (m_colorMatcher.matchClosestColor(cSensor.getColor()).color != c) {
            cWheelMotor.set(speed);
            // If it tries for more than 10 seconds, stop the loop
            if (Timer.getFPGATimestamp() - start > 10)
                break;
        }
        stopColorWheel();
        Robot.led.setBlink(96, 200);
    }

    /**
     * Stop the color wheel motor
     */
    public void stopColorWheel() {
        cWheelMotor.set(0);
    }

    private String getCurrentColor() {

        String colorString = "";
        ColorMatchResult match = m_colorMatcher.matchClosestColor(cSensor.getColor());

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }

        return colorString;

    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putString("Color Sensor", getCurrentColor());
        SmartDashboard.putNumber("Color Count", count);
    }

    @Override
    public void zeroSensors() {
        count = 0;
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
