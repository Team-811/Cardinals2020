/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto.CrossLine;
import frc.robot.commands.Auto.CrossLineAndShootComp;
import frc.robot.controllers.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeStorage;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static Shooter shooter;
  public static IntakeStorage intakeStorage;
  public static OI controllers;
  public static Vision vision;
  public static LED led;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    drivetrain = Drivetrain.getInstance();
    shooter = Shooter.getInstance();
    intakeStorage = IntakeStorage.getInstance();
    controllers = OI.getInstance();
    vision = Vision.getInstance();
    led = LED.getInstance();

    vision.initialize();

    drivetrain.zeroSensors();

    updateSmartdashboard();

    //set default LED pattern
    led.setRainbowGradient();

    CameraServer server = CameraServer.getInstance();
    server.startAutomaticCapture();

    // List auto options here and assign them each to a command. This will put them
    // in SmartDashboard and allow them to be selected
    m_chooser.setDefaultOption("Cross Line", new CrossLine());
    m_chooser.addOption("Cross Line and Shoot (Middle)", new CrossLineAndShootComp());
    m_chooser.addOption("Do Nothing", null);

    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    // drivetrain.resetSubsystem();
    // shooter.resetSubsystem();
    // intakeStorage.resetSubsystem();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard.
   */
  @Override
  public void autonomousInit() {

    // See which command we selected
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateSmartdashboard();
  }

  /**
   * This function is called when teleop starts. Use it to make sure anything that
   * may have been running it auto stops
   */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }

    intakeStorage.stopIntakeStorage();
    shooter.stopShooter();

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

  private void updateSmartdashboard() {
    drivetrain.outputSmartdashboard();
    shooter.outputSmartdashboard();
    intakeStorage.outputSmartdashboard();
    controllers.outputSmartDashboard();
    vision.outputSmartdashboard();
  }

}
