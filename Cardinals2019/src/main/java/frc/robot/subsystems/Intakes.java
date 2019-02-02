/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Robot;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.
 */
public class Intakes extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid retractPiston;
  private DoubleSolenoid hatchPiston;

  private DoubleSolenoid cargoPiston;
  private TalonSRX cargoMotor;

  public Intakes()
  {
      retractPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_RETRACT_PISTON_FORWARD, Robot.robotMap.INTAKE_RETRACT_PISTON_REVERSE);
      hatchPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_HATCH_PISTON_FORWARD, Robot.robotMap.INTAKE_HATCH_PISTON_REVERSE);
      cargoPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_CARGO_PISTON_FORWARD, Robot.robotMap.INTAKE_CARGO_PISTON_REVERSE);
      cargoMotor = new TalonSRX(Robot.robotMap.INTAKE_CARGO_MOTOR);
  }

  public void retractHatchIntake()
  {
      retractPiston.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendHatchIntake()
  {
      retractPiston.set(DoubleSolenoid.Value.kForward);
  }

   public void grabHatch()
  {
      hatchPiston.set(DoubleSolenoid.Value.kForward);
  }

   public void releaseHatch()
  {
      hatchPiston.set(DoubleSolenoid.Value.kReverse);
  }

   public void bringUpCargoIntake()
  {
      cargoPiston.set(DoubleSolenoid.Value.kReverse);
  }

   public void dropCargoIntake()
  {
      cargoPiston.set(DoubleSolenoid.Value.kForward);
  }

   public void intakeCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, 1);
      
  }

   public void releaseCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stopCargo()
  {
      cargoMotor.set(ControlMode.PercentOutput, 0);
  }




  @Override
  public void outputSmartdashboard() 
  {
    
  }

  @Override
  public void zeroSensors() 
  {
      //There are no sensors
  }

  @Override
  public void stopSubsystem() 
  {
    
  }

  @Override
  public void checkSubsystem()
  {
    
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
