/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot.
 */

public class IntakeStorage extends Subsystem implements ISubsystem {

    private static IntakeStorage instance = new IntakeStorage();

    public static IntakeStorage getInstance() {
        return instance;
    }

    private TalonSRX intakeMotor;
    private TalonSRX kickerMotor;    

    public IntakeStorage() {

        intakeMotor = new TalonSRX(RobotMap.INTAKE_STORAGE);
        kickerMotor = new TalonSRX(RobotMap.KICKER);        

        zeroSensors();
        configureTalons();
    }

    boolean isRunningIntake = false;

    private void RunIntakeStorage(double speed) {
        if(isRunningIntake)
            intakeMotor.set(ControlMode.PercentOutput, speed);
        else
            intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void ToggleIntakeStorage(double speed)
    {
        isRunningIntake = !isRunningIntake;
        RunIntakeStorage(speed);
    }    

    boolean isRunningKicker = false;

    private void RunKicker(double speed) {
        if(isRunningKicker)
            kickerMotor.set(ControlMode.PercentOutput, speed);
        else
            kickerMotor.set(ControlMode.PercentOutput, 0);
    }

    public void ToggleKicker(double speed)
    {
        isRunningKicker = !isRunningKicker;
        RunKicker(speed);
    }   

    private void configureTalons() {        
        intakeMotor.setInverted(false);
        kickerMotor.setInverted(false);

        intakeMotor.setNeutralMode(NeutralMode.Brake);
        kickerMotor.setNeutralMode(NeutralMode.Brake);
    }    

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Intake Running", isRunningIntake);
        SmartDashboard.putBoolean("Kicker Running", isRunningKicker);
    }

    @Override
    public void zeroSensors() {
        
    }

    @Override
    public void resetSubsystem() {
        zeroSensors();
        configureTalons();
    }

    @Override
    public void testSubsystem() {

    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand();
    }
}
