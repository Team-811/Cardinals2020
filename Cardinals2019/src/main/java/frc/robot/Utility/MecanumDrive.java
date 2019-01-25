/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

/**
 * Add your docs here.
 */
public class MecanumDrive 
{
    private Output driveOutput;

    private int forwardBackwardInvert;
    private int rotationInvert;
    private int strafingInvert;

    public MecanumDrive()
    {
        driveOutput = new Output();

        forwardBackwardInvert = 1;
        rotationInvert = 1;
        strafingInvert = 1;
    }

    public Output tankMecanumDrive(double leftValue, double rightValue, double strafeValue, double deadbandValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        leftValue = leftValue * forwardBackwardInvert;
        rightValue = leftValue * rotationInvert;
        strafeValue = strafeValue * strafingInvert;

        double topLeftValue = (leftValue - strafeValue);
        double bottomLeftValue = (leftValue + strafeValue);
        double topRightValue = (rightValue + strafeValue);
        double bottomRightValue = (rightValue - strafeValue);

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }


    public Output arcadeMecanumDrive(double forwardValue, double rotationValue, double strafeValue)
    {
        //First invert all the values if needed before doing the motor output calculations
        forwardValue = forwardValue * forwardBackwardInvert;
        strafeValue = strafeValue * strafingInvert;
        rotationValue = rotationValue * rotationInvert;


        double topLeftValue = ((forwardValue + rotationValue) - strafeValue);
        double bottomLeftValue = ((forwardValue + rotationValue) + strafeValue);
        double topRightValue = ((forwardValue - rotationValue) + strafeValue);
        double bottomRightValue = ((forwardValue - rotationValue) - strafeValue);

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    //TODO
    public Output curvatureMecanumDrive(double forwardValue, double rotationValue, boolean isQuickTurn, boolean isHighGear, double strafeValue, double deadbandValue)
    {
        double m_quickStopThreshold = 0.2;
        double m_quickStopAlpha = 0.1;
        double m_quickStopAccumulator = 0;

        forwardValue = limit(forwardValue, 1);
        forwardValue = handleDeadband(forwardValue, deadbandValue);

        rotationValue = limit(rotationValue, 1);
        rotationValue = handleDeadband(rotationValue, deadbandValue);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(forwardValue) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * limit(rotationValue, 1) * 2;
      }
      overPower = true;
      angularPower = rotationValue;
    } else {
      overPower = false;
      angularPower = Math.abs(forwardValue) * rotationValue - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = forwardValue + angularPower;
    double rightMotorOutput = forwardValue - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

        double topLeftValue = leftMotorOutput + strafeValue;
        double bottomLeftValue = leftMotorOutput - strafeValue;
        double topRightValue = rightMotorOutput - strafeValue;
        double bottomRightValue = rightMotorOutput + strafeValue; 

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }


     
    public Output fieldOrientedDrive(double forwardValue, double strafeValue, double rotationValue, double gyroAngle)
    {
        //First invert all the values if needed before doing the motor output calculations
        forwardValue = forwardValue * forwardBackwardInvert;
        strafeValue = strafeValue * strafingInvert;
        rotationValue = rotationValue * rotationInvert;

        //Calculate the correct forward and strafe values based on the robot's heading
        double y = Math.sin(Math.toRadians(gyroAngle));
        double x= Math.cos(Math.toRadians(gyroAngle));
        double forward= forwardValue*x + strafeValue*y;
        double strafe= -forwardValue*y + strafeValue*x;

        //Calculates the output of each motor 
        double topLeftValue = (forward + rotationValue) + strafe;
        double bottomLeftValue = (forward + rotationValue) - strafe;
        double topRightValue = (forward - rotationValue) - strafe;
        double bottomRightValue = (forward - rotationValue) + strafe;

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }


    //Inversion methods
    public void invertForwardBackward(boolean inverted)
    {
        if(inverted)
        {
            forwardBackwardInvert = -1;
        }
        else
        {
            forwardBackwardInvert = 1;
        }
    }

    public void invertRotation(boolean inverted)
    {
        if(inverted)
        {
            rotationInvert = -1;
        }
        else
        {
            rotationInvert = 1;
        }
    }

    public void invertStrafing(boolean inverted)
    {
        if(inverted)
        {
            strafingInvert = -1;
        }
        else
        {
            strafingInvert = 1;
        }
    }


    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    private double limit(double input, double limit)
    {
        if(input > limit )
        {
            return limit;
        }
        else if(input < -limit)
        {
            return -limit;
        }
        else
        {
            return input;
        }
    }
        
}


    

