/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem deals with the color wheel and contains methods to
 * automatically perform both rotation and position control
 */

public class ColorWheel extends Subsystem implements ISubsystem {

    private static ColorWheel instance = new ColorWheel();

    public static ColorWheel getInstance() {
        return instance;
    }

    public ColorWheel() {
        cWheelMotor = new TalonSRX(RobotMap.WHEEL);
        cSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    private TalonSRX cWheelMotor;
    private ColorSensorV3 cSensor;

    /**
     * Rotate the color wheel 4 times. Blink LEDs green when complete
     * 
     * @param speed (0-1)
     */
    public void rotationControl(double speed) {
        Color initial = cSensor.getColor();
        int count = 0;

        // the color sensor will see the same color 8 times during one rotation
        while (count < 8) {
            cWheelMotor.set(ControlMode.PercentOutput, speed);
            if (cSensor.getColor().equals(initial)) {
                count++;
                outputSmartdashboard();
            }
        }
        stopColorWheel();
        Robot.led.setBlink(96, 200);
    }

    /**
     * Spins the color wheel to the given color, Blinks LEDs green when complete
     * @param speed (0-1)
     */
    public void positionControl(double speed) {
        String target = Robot.ds.getGameSpecificMessage();

        if (target.length() > 0) {
            switch (target.charAt(0)) {
            case 'B':
                // Blue case code
                break;
            case 'G':
                // Green case code
                break;
            case 'R':
                // Red case code
                break;
            case 'Y':
                // Yellow case code
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
     * @param c
     * @param speed
     */
    private void goToColor(Color c, double speed) {
        while(cSensor.getColor()!=c)
        {
            cWheelMotor.set(ControlMode.PercentOutput, speed);
        }
        stopColorWheel();
        Robot.led.setBlink(96, 200);
    }

    /**
     * Stop the color wheel motor
     */
    public void stopColorWheel() {
        cWheelMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putString("Color Sensor", cSensor.getColor().toString());
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
