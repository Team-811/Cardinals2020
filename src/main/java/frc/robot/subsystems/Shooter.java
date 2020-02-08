/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This is a subsystem class. A subsystem interacts with the hardware components
 * on the robot.
 */

public class Shooter extends Subsystem implements ISubsystem {

    private static Shooter instance = new Shooter();

    public static Shooter getInstance() {
        return instance;
    }

    private CANSparkMax shooterMotor;
    private CANEncoder shooterEncoder;

    public Shooter() {
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();

        zeroSensors();
        configureSparkMAX();
    }

    boolean isRunning = false;

    private void RunShooter(double speed) {
        if(isRunning)
            shooterMotor.set(speed);
        else
            shooterMotor.set(0);
    }

    public void ToggleShooter(double speed)
    {
        isRunning = !isRunning;
        RunShooter(speed);
    }

    public double getShooterEncoder() {
        return shooterEncoder.getPosition();
    }

    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    private void configureSparkMAX() {
        zeroEncoders();
        shooterMotor.setInverted(true);
        shooterMotor.setIdleMode(IdleMode.kBrake);
    }

    private void zeroEncoders() {
        shooterEncoder.setPosition(0);
    }

    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Shooter Running", isRunning);
        SmartDashboard.putNumber("Shooter Encoder ", getShooterEncoder());
        SmartDashboard.putNumber("Shooter Velocity ", getShooterVelocity());
    }

    @Override
    public void zeroSensors() {
        zeroEncoders();
    }

    @Override
    public void resetSubsystem() {
        zeroSensors();
        configureSparkMAX();
        RunShooter(0);
        isRunning = false;
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
