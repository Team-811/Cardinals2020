/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Add your docs here.
 */
public interface ISubsystem 
{
    public abstract void outputSmartdashboard();
    public abstract void zeroSensors();
    public abstract void stopSubsystem();
    public abstract void testSubsystem();
    public abstract void checkSubsystem();
}
