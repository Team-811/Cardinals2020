/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.OI;
import frc.robot.subsystems.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static Intakes intakes;
  public static Climber climber;
  public static Elevator elevator;
  public static LED led;
  public static Vision vision;
  public static OI controllers; 

  //Command m_autonomousCommand;
  //SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    intakes = new Intakes();
    climber = new Climber();
    elevator = new Elevator();
    led = new LED();
    vision = new Vision();
    controllers = new OI();

    updateSmartdashboard();
    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);

    //CameraServer for SmartDashboard
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);

    //Start threads
    Thread networkThread = new Thread(new RunNetwork());
    networkThread.start();

    // Thread robotPosition = new Thread(new UpdateRobotPostion());
    // robotPosition.start();



    //Robot.intakes.releaseHatch();
    Robot.intakes.bringUpCargoIntake();

    led.setLEDs(9); //Red Flame
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      updateSmartdashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
    //m_autonomousCommand.start();
    //}

    updateSmartdashboard();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    updateSmartdashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
   // if (m_autonomousCommand != null) {
   //   m_autonomousCommand.cancel();
    //}

      updateSmartdashboard();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    updateSmartdashboard();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testInit() {
    //Initial Print
    System.out.println("/////////////////////////////////////////////////////");
    System.out.println("***********Robot has now entered test mode***********");
    System.out.println("*****Warning: Robot will move durig this period*****");
    System.out.println("*******Please move out of the way of the Robot*******");
    System.out.println("/////////////////////////////////////////////////////");
    Timer.delay(10);

    //Run tests
    drivetrain.testSubsystem();
    elevator.testSubsystem();
    intakes.testSubsystem();


    
  }


  private void updateSmartdashboard()
  {
      drivetrain.outputSmartdashboard();
      elevator.outputSmartdashboard();
      intakes.outputSmartdashboard();
      //led.outputSmartdashboard();
      vision.outputSmartdashboard();
  }











  public class RunNetwork implements Runnable
  {
    public void run()
    {
      while (true) {
        try {
            Thread.sleep(20L);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(led.instanceCounter == 0)
        {
          led = new LED();
        }

      }
    }
  }

  public class UpdateRobotPostion implements Runnable
  {
    public void run()
    {
      
    }
  }

  

}
