/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

/**
 * This class is used to store a X coordinate, a Y coordinate, and an angle of something.  Also, it is used to store the robot's
 * position on the field based on its starting position, encoder values, and gyro values
 */

public class Odometry 
{
    private double xCoordinate;
    private double yCoordinate;
    private double theta; //in radians

    private double initialXCoordinate;
    private double initialYCoordinate;
    private double initialTheta;

    private double initialEncoderLeft;
    private double initialEncoderRight;
    private double initialGyroAngle;

    //Default Constructor
    public Odometry()
    {
        xCoordinate = 0;
        yCoordinate = 0;
        theta = 0;

        initialXCoordinate = 0;
        initialYCoordinate = 0;
        initialTheta = 0;


    }

    //Parameterized constructors
    public Odometry(double xCoordinate, double yCoordinate, double theta, boolean inRadians)
    {
        if(!inRadians)
        {
            theta = Math.toRadians(theta);
        }

        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.theta = theta;

        initialXCoordinate = 0;
        initialYCoordinate = 0;
        initialTheta = 0;
    }

    
    //The following deal with basic coordinate storage 


    //Getter methods

    public double getX ()
    {
        return xCoordinate;
    }

    public double getY()
    {
        return yCoordinate;
    }

    public double getThetaInRadians()
    {
        return theta;
    }

    public double getThetaInDegrees()
    {
        return Math.toDegrees(theta);
    }



    //Setter Methods

    public void setX(double xCoordinate)
    {
        this.xCoordinate = xCoordinate;
    }

    public void setY(double yCoordinate)
    {
        this.yCoordinate = yCoordinate;
    }

    public void setThetaRadians(double thetaRadians)
    {
        this.theta = thetaRadians;
    }

    public void setThetaDegrees(double thetaDegrees)
    {
        this.theta = Math.toRadians(thetaDegrees);
    }







    //The following deal with the position of the robot


    //Setting Odometries based on robot sensors

    public void setRobotOdometry(double encoderLeft, double encoderRight, double gyroAngle, boolean inRadians)
    {
        if(!inRadians)
        {
            gyroAngle = Math.toRadians(gyroAngle);
        }

        this.xCoordinate = (Math.cos(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2)) + initialXCoordinate;
        this.yCoordinate = (Math.sin(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2)) + initialYCoordinate;
        this.theta = getDeltaGyro(gyroAngle) + initialTheta; 
    }

    public void setInitialRobotOdometry(double x, double y, double theta, boolean inRadians)
    {
        if(!inRadians)
        {
            theta = Math.toRadians(theta);
        }

        this.initialXCoordinate = x;
        this.initialYCoordinate = y;
        this.initialTheta = theta;
    }

    public void setInitialRobotState(double initialEncoderLeft, double initialEncoderRight, double initialGyroAngle, boolean inRadians)
    {
        if(!inRadians)
        {
            initialGyroAngle = Math.toRadians(theta);
        }

        this.initialEncoderLeft = initialEncoderLeft;
        this.initialEncoderRight = initialEncoderRight;
        this.initialGyroAngle = Math.toRadians(initialGyroAngle);

    }

    private double getDeltaEncoderLeft(double encoderLeft)
    {
        return encoderLeft - initialEncoderLeft;
    }

    private double getDeltaEncoderRight(double encoderRight)
    {
        return encoderRight - initialEncoderRight;
    }

    private double getDeltaGyro(double gyroAngle)
    {
        return gyroAngle - initialGyroAngle;
    }


}
