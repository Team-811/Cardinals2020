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
import frc.robot.RobotMap;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot. This subsystem deals with vision tracking with the Pixy2 Camera
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
    }

    private TalonFX winch;
    private TalonSRX telescopeExtend;
    private TalonSRX telescopeWinch;

    private DigitalInput telescopeUp;
    private DigitalInput telescopeHigh;
    private DigitalInput telescopeLow;

    /*
     * Limit Switch Patterns:
     * 
     * telescopeHigh on and telescopeLow on: telescope extend up only 
     * telescopeHigh on and telescopeLow off: telescope extend up or down 
     * telescopeHigh off andtelescopeLow off: telescope extend down only 
     * telescopeHigh off and telescope low on: do not move, wiring issue
     * 
     * telescopeUp on: stop telescope winch
     * 
     */

    public void runWinch(double speed) {
        winch.set(ControlMode.PercentOutput, speed);
    }

    public void runTelescopeExtend(double speed) {
        if (speed < 0) {
            if (telescopeHigh.get() && telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, 0);
            else if (telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed);
            else if (!telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed);
            else
                telescopeExtend.set(ControlMode.PercentOutput, 0);

        } else {
            if (telescopeHigh.get() && telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed);
            else if (telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, speed);
            else if (!telescopeHigh.get() && !telescopeLow.get())
                telescopeExtend.set(ControlMode.PercentOutput, 0);
            else
                telescopeExtend.set(ControlMode.PercentOutput, 0);
        }
    }

    public void runTelescopeWinch(double speed) {
        if (!telescopeUp.get())
            telescopeWinch.set(ControlMode.PercentOutput, speed);
        else
            telescopeWinch.set(ControlMode.PercentOutput, 0);
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
