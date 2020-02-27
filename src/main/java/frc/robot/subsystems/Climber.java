/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem deals with the climber
 */

public class Climber extends Subsystem implements ISubsystem {

    private static Climber instance = new Climber();

    public static Climber getInstance() {
        return instance;
    }

    public Climber() {
        winch = new TalonFX(RobotMap.WINCH);
        telescopeExtend = new TalonSRX(RobotMap.TELESCOPE_EXTEND);
        telescopeWinch = new TalonSRX(RobotMap.TELESCOPE_WINCH);

        telescopeUp = new DigitalInput(RobotMap.TELESCOPE_UP);
        telescopeHigh = new DigitalInput(RobotMap.TELESCOPE_HIGH);
        telescopeLow = new DigitalInput(RobotMap.TELESCOPE_LOW);

        zeroSensors();
    }

    private TalonFX winch;
    private TalonSRX telescopeExtend;
    private TalonSRX telescopeWinch;

    private boolean winchRunning = false;
    private boolean tUp = false;
    private boolean tHigh = false;
    private boolean tLow = false;

    private DigitalInput telescopeUp;
    private DigitalInput telescopeHigh;
    private DigitalInput telescopeLow;

    private double telescopeExtendMax = 0;
    private int direction = 1;
    private boolean isReverse = false;

    /**
     * Run the winch motor at a given speed
     * 
     * @param speed (0-1)
     */
    public void runWinch(double speed) {
        winch.set(ControlMode.PercentOutput, speed * direction);
        if (speed == 0) {
            winchRunning = false;
            Robot.setDefaultLED();
        } else {
            winchRunning = true;
            Robot.led.setRainbowChase(40);
        }
    }

    /*
     * Limit Switch Patterns:
     * 
     * telescopeHigh on and telescopeLow on: telescope extend up only telescopeHigh
     * on and telescopeLow off: telescope extend up or down telescopeHigh off
     * andtelescopeLow off: telescope extend down only telescopeHigh off and
     * telescope low on: do not move, wiring issue
     * 
     * telescopeUp on: stop telescope winch
     * 
     * These limit switches prevent certain motors from moving once they have
     * reached a certain point to avoid damaging the robot
     * 
     */

    /**
     * Run the telescope arm extender. This accounts for limit switches and will
     * only run the motor in allowed configurations and will blink red LEDs if
     * switches are hit
     * 
     * @param speed (0-1)
     */
    public void runTelescopeExtend(double speed) {
        if (speed * direction < 0) {
            if (telescopeHigh.get() && telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, 0);
            else if (telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed * direction);
            else if (!telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed * direction);
            else
                telescopeExtend.set(ControlMode.PercentOutput, 0);

        } else {
            if (telescopeHigh.get() && telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed * direction);
            else if (telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed * direction);
            else if (!telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, 0);
            else
                telescopeExtend.set(ControlMode.PercentOutput, 0);
        }

        if (telescopeExtend.getSelectedSensorPosition() >= telescopeExtendMax
                || telescopeExtend.getSelectedSensorPosition() < 0) {
            telescopeExtend.set(ControlMode.PercentOutput, 0);
            Robot.led.setBlink(5, 200);
        }

    }

    /**
     * Run the telescope arm winch to bring it up perpendicular to the ground.
     * Accounts for limit swiches and blinks red LEDs if they are hit
     * 
     * @param speed (0-1)
     * @return true when the limit switch has been hit
     */
    public boolean runTelescopeWinch(double speed) {
        if (!telescopeUp.get()) {
            telescopeWinch.set(ControlMode.PercentOutput, speed * direction);
            return false;
        } else {
            telescopeWinch.set(ControlMode.PercentOutput, 0);
            Robot.led.setBlink(5, 200);
            return true;
        }
    }

    /**
     * Set the climber mechanisms to move in reverse if needed
     * 
     * @param state true for reverse
     */
    public void reverse(boolean state) {
        if (state) {
            isReverse = true;
            direction = -1;
        } else {
            isReverse = false;
            direction = 1;
        }
    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Winch Running", winchRunning);
        SmartDashboard.putBoolean("THigh Limit Switch", tHigh);
        SmartDashboard.putBoolean("TLow Limit Switch", tLow);
        SmartDashboard.putBoolean("TUp Limit Switch", tUp);        
        SmartDashboard.putBoolean("Climber Reverse", isReverse);
    }

    @Override
    public void zeroSensors() {
        telescopeExtend.setSelectedSensorPosition(0);
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
