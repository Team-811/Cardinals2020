/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.robot;

/**
 * Add your docs here.
 */
public class Intakes extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid retractPiston;
  private DoubleSolenoid hatchPiston;

  private DoubleSolenoid cargoPiston;
  private TalonSRX cargoMotor;

  public Intakes()
  {
      retractPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_RETRACT_PISTON, Robot.robotMap.INTAKE_RETRACT_PISTON_FORWARD, Robot.robotMap.INTAKE_RETRACT_PISTON_BACKWARD);
      hatchPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_HATCH_PISTON, Robot.robotMap.INTAKE_HATCH_PISTON_FORWARD, Robot.robotMap.INTAKE_HATCH_PISTON_BACKWARD);
      cargoPiston = new DoubleSolenoid(Robot.robotMap.INTAKE_CARGO_PISTON, Robot.robotMap.INTAKE_CARGO_PISTON_FORWARD, Robot.robotMap.INTAKE_CARGO_PISTON_BACKWARD);
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
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
