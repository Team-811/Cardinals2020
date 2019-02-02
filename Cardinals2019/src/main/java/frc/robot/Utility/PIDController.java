/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

/**
 * This class contains a pid controller.  A PID controller essentialy helps a mechanism go to a certain position by 
 * slowing the the mechanism down as it approaches the goal.  The P gain controls how fast you approach the object.
 * The I gain controls how much you want to boost the mechanism if it cant quite reach the desired position.  
 * The D gains dampens the mechanism if it oscilates around the goal.
 *
 */

public class PIDController {

    private double error;
    private double lastError;
    
    private double integral;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    
    private final double DT = 0.02; // In seconds so 20ms
    
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
    
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }
    
    public double updatePID(double position, double goal) {
        error = goal - position;
        integral += error * DT;
        double output = (kP * error) + (kI * integral) + (kD * (error - lastError) / DT) + kF;

        //Keeps output between -1 and 1 for motor control
        if(output >= 1)
            output = 1;
        else if(output <= -1)
            output = -1;
        
        lastError = error;
        return output;
    }
    
    public void resetController() {
        this.lastError = 0.0;
        this.error = 0.0;
        this.integral = 0.0;
    }
    
    public void updateConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    
    public double getError() {
        return lastError;
    }
}










