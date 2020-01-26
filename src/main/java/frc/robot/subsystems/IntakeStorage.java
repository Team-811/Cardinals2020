/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private CANSparkMax intakeMotor;
    private CANEncoder intakeEncoder;

    private CANSparkMax kickerMotor;
    private CANEncoder kickerEncoder;

    public IntakeStorage() {

        intakeMotor = new CANSparkMax(RobotMap.INTAKE_STORAGE, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();

        kickerMotor = new CANSparkMax(RobotMap.KICKER, MotorType.kBrushless);
        kickerEncoder = kickerMotor.getEncoder();

        zeroSensors();
        configureSparkMAX();
    }

    boolean isRunningIntake = false;

    private void RunIntakeStorage(double speed) {
        if(isRunningIntake)
            intakeMotor.set(speed);
        else
            intakeMotor.set(0);
    }

    public void ToggleIntakeStorage(double speed)
    {
        isRunningIntake = !isRunningIntake;
        RunIntakeStorage(speed);
    }

    public double getIntakeEncoder() {
        return intakeEncoder.getPosition();
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    boolean isRunningKicker = false;

    private void RunKicker(double speed) {
        if(isRunningKicker)
            kickerMotor.set(speed);
        else
            kickerMotor.set(0);
    }

    public void ToggleKicker(double speed)
    {
        isRunningKicker = !isRunningKicker;
        RunKicker(speed);
    }

    public double getKickerEncoder() {
        return kickerEncoder.getPosition();
    }

    public double getKickerVelocity() {
        return kickerEncoder.getVelocity();
    }

    private void configureSparkMAX() {
        zeroEncoders();
        intakeMotor.setInverted(false);
        kickerMotor.setInverted(false);
    }

    private void zeroEncoders() {
        intakeEncoder.setPosition(0);
        kickerEncoder.setPosition(0);
    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Intake Running", isRunningIntake);
        SmartDashboard.putNumber("Intake Encoder ", getIntakeEncoder());
        SmartDashboard.putNumber("Intake Velocity ", getIntakeVelocity());

        SmartDashboard.putBoolean("Kicker Running", isRunningKicker);
        SmartDashboard.putNumber("Kicker Encoder ", getKickerEncoder());
        SmartDashboard.putNumber("Kicker Velocity ", getKickerVelocity());
    }

    @Override
    public void zeroSensors() {
        zeroEncoders();
    }

    @Override
    public void resetSubsystem() {
        zeroSensors();
        configureSparkMAX();
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
