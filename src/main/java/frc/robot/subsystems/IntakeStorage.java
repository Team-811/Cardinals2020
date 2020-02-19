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
 * This is the subsystem class for the Robot intake and storage container. It
 * contains methods to start/stop the intake at given speeds. The intake and
 * storage are connected together, so only one motor is used to control the two.
 */

public class IntakeStorage extends Subsystem implements ISubsystem {

    private static IntakeStorage instance = new IntakeStorage();

    /**
     * @return A new instance of the Intake/Storage
     */
    public static IntakeStorage getInstance() {
        return instance;
    }

    private CANSparkMax intakeMotor;
    private CANEncoder intakeEncoder;

    private boolean intakeIsRunning = false;

    public boolean intakeRunning()
    {
        return intakeIsRunning;
    }

    /**
     * @return A new instance of the Intake/Storage
     */
    public IntakeStorage() {
        intakeMotor = new CANSparkMax(RobotMap.INTAKE_STORAGE, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        resetSubsystem();
    }

    /**
     * This method runs the intake/storage at a given speed. NOTE: it only sets the
     * speed; it does not toggle the subsystem and will not stop unless the speed
     * parameter is 0.
     * 
     * @param speed (0-1)
     */
    public void runIntakeStorage(double speed) {
        intakeMotor.set(speed);
        if (speed == 0) {
            intakeIsRunning = false;
        } else
            intakeIsRunning = true;
    }

    /**
     * This method toggles the intake/storage at a given speed. If it is on, it
     * turns off and vice versa.
     * 
     * @param speed (0-1)
     */
    public void toggleIntakeStorage(double speed) {
        intakeIsRunning = !intakeIsRunning;

        if (intakeIsRunning)
            intakeMotor.set(speed);
        else
            intakeMotor.set(0);
    }

    /**
     * Stops the intake/storage
     */
    public void stopIntakeStorage() {
        intakeIsRunning = false;
        intakeMotor.set(0);
    }

    /**
     * 
     * @return Speed of the intake
     */
    public double getIntakeStorageVelocity()
    {
        return intakeEncoder.getVelocity();
    }

    /**
     * Configures subsytem-specific settings for motor controllers
     */
    private void configureMotorControllers() {
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Outputs Intake/Storage subsystem information to SmartDashboard for drivers.
     */
    @Override
    public void outputSmartdashboard() {
        SmartDashboard.putBoolean("Intake Running", intakeIsRunning);
        SmartDashboard.putNumber("Intake Velocity", getIntakeStorageVelocity());
    }

    /**
     * Stops the intake/storage and reconfigures motor controllers.
     */
    @Override
    public void resetSubsystem() {
        stopIntakeStorage();
        zeroSensors();
        configureMotorControllers();
    }

    @Override
    public void zeroSensors() {
        intakeEncoder.setPosition(0);
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
