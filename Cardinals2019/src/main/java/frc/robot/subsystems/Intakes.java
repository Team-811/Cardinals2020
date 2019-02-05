/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;
import frc.robot.lib.TalonChecker;

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
      retractPiston = new DoubleSolenoid(RobotMap.INTAKE_RETRACT_PISTON_FORWARD, RobotMap.INTAKE_RETRACT_PISTON_REVERSE);
      hatchPiston = new DoubleSolenoid(RobotMap.INTAKE_HATCH_PISTON_FORWARD, RobotMap.INTAKE_HATCH_PISTON_REVERSE);
      cargoPiston = new DoubleSolenoid(RobotMap.INTAKE_CARGO_PISTON_FORWARD, RobotMap.INTAKE_CARGO_PISTON_REVERSE);
      cargoMotor = new TalonSRX(RobotMap.INTAKE_CARGO_MOTOR);
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
      
  }

  @Override
  public void resetSubsystem() 
  {
    retractHatchIntake();
    releaseHatch();
    bringUpCargoIntake();
    stopCargo();
  }

  @Override
  public void checkSubsystem()
  {
        
  }

  @Override
  public void testSubsystem() 
  {
        boolean sucess = true;

        Timer delay = new Timer();
        System.out.println("///////////////////////////////////////////////////");
        System.out.println("***************Beginning Intake Test***************");
        Timer.delay(0.2);

        //Test Hatch Piston
        System.out.println("Extending Hatch Piston");
        Timer.delay(0.5);
        grabHatch();
        Timer.delay(0.2);

        System.out.println("Closing Hatch Piston");
        Timer.delay(0.5);
        releaseHatch();
        Timer.delay(0.2);

        //Test Hatch Extension Piston
        System.out.println("Extending Hatch Intake");
        Timer.delay(0.5);
        grabHatch();
        Timer.delay(0.2);

        System.out.println("Retracting Hatch Intake");
        Timer.delay(0.5);
        grabHatch();
        Timer.delay(0.2);

        //Test Cargo Piston
        System.out.println("Bring Down Cargo Intake");
        Timer.delay(0.5);
        dropCargoIntake();
        Timer.delay(0.2);

        System.out.println("Bring up Cargo Intake");
        Timer.delay(0.5);
        bringUpCargoIntake();
        Timer.delay(0.2);

        System.out.println("Checking Cargo Motor");
        Timer.delay(0.5);
        TalonChecker checker = new TalonChecker("Cargo Talon", cargoMotor, false);
        sucess = checker.runTest(5); //TODO
        Timer.delay(0.2);

        if(sucess)
            System.out.println("***************Everything in intake is working***************");
        else
            System.out.println("***************Error in Intake***************");


        
        
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
