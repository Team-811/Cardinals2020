/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

/*
public class Elevator extends Subsystem implements ISubsystem{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum LiftState {
    GoingUp,
    GoingDown,
    Stationary,
    BottomedOut,
    ToppedOut,
}

public LiftState getState() {
    return state;
}

private void setState(LiftState newState) {
    this.state = newState;
}

private volatile LiftState state = LiftState.Stationary;

private volatile TalonSRX rightElevatorLeader = new TalonSRX(RobotMap.RIGHT_ELEVATOR_MAIN);
private TalonSRX rightElevatorSlave = new TalonSRX(RobotMap.RIGHT_ELEVATOR_SLAVE);
private TalonSRX leftElevatorSlaveA = new TalonSRX(RobotMap.LEFT_ELEVATOR_SLAVE_A);
private TalonSRX leftElevatorSlaveB = new TalonSRX(RobotMap.LEFT_ELEVATOR_SLAVE_B);

private double kP = 0.5; // .3
private double kI = 0.0;
private double kD = 4.0; // 4.0
private double kF = 0.1165 * 2; // 0.1165 * 2

//81.2 inches per/s
//Gear is 42:26 geared up
//4096 ticks per rev
//233 ticks per inch
private static final int CRUISE_VELOCITY = 17600; // 1024
private static final int CRUISE_ACCELERATION = 11000; // 1024
private static final int CRUISE_VELOCITY_DOWN = (int) (CRUISE_VELOCITY * 0.7); // 1024
private static final int CRUISE_ACCELERATION_DOWN = (int) (CRUISE_ACCELERATION * 0.6); // 1024

public enum Positions {
    Intake(300),
    ScoreSwitch(50000),
    ScoreScale(22000),
    PreClimb(70000),
    Top(75001),
    ChinUp(0),
    ClimbingBar(0);
    private int position;

    Positions(int encPos) {
        this.position = encPos;
    }

    public int getPosition() {
        return this.position;
    }
}

private volatile Positions position = Positions.Intake;

public Positions getPosition() {
    return position;
}

private void setPosition(Positions newPos) {
    this.position = newPos;
}

private final int MOTION_MAGIC_TOLERANCE = 150;
private static final double ELEVATOR_HI_POW = 1.0;
private static final double ELEVATOR_LOW_POW = -ELEVATOR_HI_POW;

public Elevator() {
    this.rightElevatorSlave.follow(rightElevatorLeader);
    this.leftElevatorSlaveA.follow(rightElevatorLeader);
    this.leftElevatorSlaveB.follow(rightElevatorLeader);
    this.rightElevatorLeader.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.rightElevatorLeader.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.rightElevatorLeader.configNominalOutputForward(0.0, 0);
    this.rightElevatorLeader.configNominalOutputReverse(0.0, 0);
    this.rightElevatorSlave.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.rightElevatorSlave.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.rightElevatorSlave.configNominalOutputForward(0.0, 0);
    this.rightElevatorSlave.configNominalOutputReverse(0.0, 0);
    this.leftElevatorSlaveA.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.leftElevatorSlaveA.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.leftElevatorSlaveA.configNominalOutputForward(0.0, 0);
    this.leftElevatorSlaveA.configNominalOutputReverse(0.0, 0);
    this.leftElevatorSlaveB.configPeakOutputForward(ELEVATOR_HI_POW, 0);
    this.leftElevatorSlaveB.configPeakOutputReverse(ELEVATOR_LOW_POW, 0);
    this.leftElevatorSlaveB.configNominalOutputForward(0.0, 0);
    this.leftElevatorSlaveB.configNominalOutputReverse(0.0, 0);

    //Encoder
    this.rightElevatorLeader.setSensorPhase(true);
    this.rightElevatorLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    resetEncoders();
    //this.rightElevatorLeader.setSelectedSensorPosition(rightElevatorLeader.getSensorCollection().getQuadraturePosition(), 0, 0);

    //Neutral mode
    this.rightElevatorLeader.setNeutralMode(NeutralMode.Brake);
    this.rightElevatorSlave.setNeutralMode(NeutralMode.Brake);
    this.leftElevatorSlaveA.setNeutralMode(NeutralMode.Brake);
    this.leftElevatorSlaveB.setNeutralMode(NeutralMode.Brake);

    this.leftElevatorSlaveA.setInverted(true);
    this.leftElevatorSlaveB.setInverted(true);
    this.rightElevatorLeader.setInverted(false);
    this.rightElevatorSlave.setInverted(false);

    configPIDF(kP, kI, kD, kF);
    configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);

    //Hard coded to false to reduce chance of breaking...
}


public void resetEncoders() {
    this.rightElevatorLeader.setSelectedSensorPosition(0, 0, 0);
}

public int getQuadraturePosition() {
    return rightElevatorLeader.getSelectedSensorPosition(0);
    //return rightElevatorLeader.getSensorCollection().getQuadraturePosition();
}

public void startMotionMagic(Positions pos) { // Up is now negative
    if (getQuadraturePosition() > pos.getPosition()) {
        setState(LiftState.GoingDown);
        configMotionMagic(CRUISE_VELOCITY_DOWN, CRUISE_ACCELERATION_DOWN);
    } else if (getQuadraturePosition() < pos.getPosition()) {
        setState(LiftState.GoingUp);
        configMotionMagic(CRUISE_VELOCITY, CRUISE_ACCELERATION);
    }

    rightElevatorLeader.set(ControlMode.MotionMagic, pos.getPosition());
}

public void checkMotionMagicTermination(Positions pos) {
    if (pos == Positions.Intake) {
        if (getQuadraturePosition() <= (MOTION_MAGIC_TOLERANCE * 2)) {
            state = LiftState.Stationary;
            stopElevator();
            position = pos;
        }
    } else if (Math.abs(pos.getPosition() - getQuadraturePosition()) <= MOTION_MAGIC_TOLERANCE) {
        state = LiftState.Stationary;
        stopElevator();
        position = pos;
    }
    SmartDashboard.putString("Desired elevator position enum", pos.toString());
    SmartDashboard.putNumber("Motion Magic set position", rightElevatorLeader.getClosedLoopTarget(0));
    SmartDashboard.putNumber("CTRError", rightElevatorLeader.getClosedLoopError(0));
    SmartDashboard.putNumber("Desired elevator position", pos.getPosition());
    SmartDashboard.putNumber("Closed loop error", Math.abs(pos.getPosition() - getQuadraturePosition()));
}

public void stopElevator() {
    rightElevatorLeader.set(ControlMode.PercentOutput, 0.0);
}

public void directElevate(double pow) {
    if (getState() == LiftState.BottomedOut && pow < 0.0) {
        return;
    }
    if (getState() == LiftState.ToppedOut && pow > 0.0) {
        return;
    }
    if (pow > 0.0) {
        setState(LiftState.GoingUp);
    }
    if (pow < 0.0) {
        setState(LiftState.GoingDown);
    }
    if (pow == 0.0) {
        setState(LiftState.Stationary);
    }
    rightElevatorLeader.set(ControlMode.PercentOutput, pow);
}

private void checkIfToppedOut() {
    if (getQuadraturePosition() >= Positions.Top.getPosition() && getState() != LiftState.GoingDown) {
        setState(LiftState.ToppedOut);
        setPosition(Positions.Top);
        stopElevator();
    }
}

private void checkIfZeroedOut() {
    if (getQuadraturePosition() <= Positions.Intake.getPosition() && getState() != LiftState.GoingUp) {
        setState(LiftState.BottomedOut);
        setPosition(Positions.Intake);
        stopElevator();
    }
}

public void configPIDF(double kP, double kI, double kD, double kF) {
    rightElevatorLeader.config_kP(0, kP, 0);
    rightElevatorLeader.config_kI(0, kI, 0);
    rightElevatorLeader.config_kD(0, kD, 0);
    rightElevatorLeader.config_kF(0, kF, 0);
}

/**
 * Set parameters for motion magic control
 *
 * @param cruiseVelocity cruise velocity in sensorUnits per 100ms
 * @param acceleration   cruise acceleration in sensorUnits per 100ms
 */

/*
public void configMotionMagic(int cruiseVelocity, int acceleration) {
    rightElevatorLeader.configMotionCruiseVelocity(cruiseVelocity, 0);
    rightElevatorLeader.configMotionAcceleration(acceleration, 0);
}

public void updatePIDFOnDashboard() {
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kF", kF);
}

public void updatePIDFFromDashboard() {
    kP = SmartDashboard.getNumber("kP", kP);
    kI = SmartDashboard.getNumber("kI", kI);
    kD = SmartDashboard.getNumber("kD", kD);
    kF = SmartDashboard.getNumber("kF", kF);
    configPIDF(kP, kI, kD, kF);
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


*/