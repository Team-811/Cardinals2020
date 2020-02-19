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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * This is the subsystem class for the Shooter. It contains methods to toggle
 * the shooter on or off and also to run the shooter automatically, accounting
 * for lost speed. This subsystem contains the shooter and kicker (storage to
 * shooter) motors.
 */
public class Shooter extends Subsystem implements ISubsystem {

    private static Shooter instance = new Shooter();

    /**
     * @return New instance of shooter subsystem.
     */
    public static Shooter getInstance() {
        return instance;
    }

    private CANSparkMax shooterMotor;
    private CANEncoder shooterEncoder;

    private TalonSRX kickerMotor;

    private boolean shooterIsRunning = false;
    private boolean kickerIsRunning = false;
    
    private double shooterFullVelocity = Constants.SHOOTER_FULL_VELOCITY;

    /**
     * @return New instance of shooter subsystem.
     */
    public Shooter() {
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();

        kickerMotor = new TalonSRX(RobotMap.KICKER);

        resetSubsystem();
    }

    /**
     * Runs the shooter at a given speed. This method accounts for speed lost when
     * shooting a ball. The kicker motor from the storage to the shooter will only
     * feed the next ball when the shooter has reached full speed again. NOTE: this
     * method does not toggle the shooter, it only adjusts the speed. It will not
     * stop the shooter unless the speed is 0. This method also runs the kicker
     * motor automatically depending on the shooter speed.
     * 
     * @param shooterSpeed (0-1)
     * @param kickerSpeed (0-1)
     */
    public void autoRunShooter(double shooterSpeed, double kickerSpeed) {
        shooterMotor.set(shooterSpeed);
        if (shooterSpeed == 0) {
            shooterIsRunning = false;

            kickerIsRunning = false;
            kickerMotor.set(ControlMode.PercentOutput, 0);
        } else {
            shooterIsRunning = true;
        }

        // only run the kicker once the shooter has reached full speed again
        if (getShooterVelocity() > shooterFullVelocity * shooterSpeed) {
            kickerIsRunning = true;
            kickerMotor.set(ControlMode.PercentOutput, kickerSpeed);
        } else {
            kickerIsRunning = false;
            kickerMotor.set(ControlMode.PercentOutput, 0);
        }
    }
    

    /**
     * Runs the shooter at a given speed. Also runs the kicker continuously.
     * 
     * @param speed (0-1)
     */
    public void runShooter(double speed) {
        shooterMotor.set(speed);
        if (speed == 0) {
            kickerMotor.set(ControlMode.PercentOutput, 0);
            kickerIsRunning = false;
            shooterIsRunning = false;
        } else {
            kickerMotor.set(ControlMode.PercentOutput, 1);
            kickerIsRunning = true;
            shooterIsRunning = true;
        }
    }

    /**
     * Toggles the shooter on/off. Each time this method is called, the shooter will
     * go into the opposite state (if on, turns off). This methods only toggles the
     * shooter, not the kicker. Mostly used for testing.
     * 
     * @param speed (0-1)
     */
    public void toggleShooter(double speed) {
        shooterIsRunning = !shooterIsRunning;
        if (shooterIsRunning)
            shooterMotor.set(speed);
        else
            shooterMotor.set(0);
    }

    /**
     * Toggles the kicker and shooter on/off. Each time this method is called, the shooter and kicker will
     * go into the opposite state (if on, turns off). Mostly used for testing.
     * 
     * @param shooterSpeed (0-1)
     * @param kickerSpeed (0-1)
     */
    public void toggleKickerAndShooter(double shooterSpeed, double kickerSpeed){
        shooterIsRunning = !shooterIsRunning;
        kickerIsRunning = !kickerIsRunning;
        if (shooterIsRunning&&kickerIsRunning)
        {
            shooterMotor.set(shooterSpeed);
            kickerMotor.set(ControlMode.PercentOutput, kickerSpeed);
        }
            
        else
        {
            shooterMotor.set(0);
            kickerMotor.set(ControlMode.PercentOutput, 0);
        }
            

    }

    /**
     * Toggles the kicker on/off. Each time this method is called, the kicker will
     * go into the opposite state (if on, turns off). This methods only toggles the
     * kicker, not the shooter. Mostly used for testing.
     * 
     * @param speed (0-1)
     */
    public void toggleKicker(double speed) {
        kickerIsRunning = !kickerIsRunning;
        if (kickerIsRunning)
            kickerMotor.set(ControlMode.PercentOutput, speed);
        else
            kickerMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Stops the shooter and kicker
     */
    public void stopShooter() {
        kickerIsRunning = false;
        shooterIsRunning = false;

        kickerMotor.set(ControlMode.PercentOutput, 0);
        shooterMotor.set(0);
    }

    /**
     * @return Speed of the shooter in RPM
     */
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Configures subsytem-specific settings for motor controllers
     */
    private void configureMotorControllers() {
        zeroSensors();

        shooterMotor.setInverted(true);
        shooterMotor.setIdleMode(IdleMode.kBrake);

        kickerMotor.setInverted(true);
        kickerMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Outputs Shooter subsystem information to SmartDashboard for drivers.
     */
    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Shooter Running", shooterIsRunning);
        SmartDashboard.putNumber("Shooter Velocity ", getShooterVelocity());

        SmartDashboard.putBoolean("Kicker Running", kickerIsRunning);
    }

    /**
     * Stops the shooter and kicker motors; reconfigures motor controllers.
     */
    @Override
    public void resetSubsystem() {
        stopShooter();
        zeroSensors();
        configureMotorControllers();
    }

    @Override
    public void zeroSensors() {
        shooterEncoder.setPosition(0);
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
