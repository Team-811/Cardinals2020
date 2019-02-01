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
/*
    //Parameterized constructors
    public Odometry(double xCoordinate, double yCoordinate, double thetaInRadians)
    {
        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.theta = theta;

        initialXCoordinate = 0;
        initialYCoordinate = 0;
        initialTheta = 0;
    }

    public Odometry(double initialXCoordinate, double initialYCoordinate, double initialTheta)
    {
        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.theta = theta;

        initialX = 0;
        initialY = 0;
        initialTheta = 0;
    }
*/

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

    public void setThetRadians(double thetaRadians)
    {
        this.theta = thetaRadians;
    }

    public void setThetaDegrees(double thetaDegrees)
    {
        this.theta = Math.toRadians(thetaDegrees);
    }


    //Setting Odometries based on robot sensors

    public void setRobotOdometry(double encoderLeft, double encoderRight, double gyroAngle)
    {
        this.xCoordinate = (Math.cos(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2)) + initialXCoordinate;
        this.yCoordinate = (Math.cos(gyroAngle) * ((getDeltaEncoderLeft(encoderLeft) + getDeltaEncoderRight(encoderRight))/2)) + initialYCoordinate;
        this.theta = (Math.toRadians(gyroAngle) - initialGyroAngle) + initialTheta;
    }

    public void setInitialRobotOdometry(double x, double y, double thetaRadians)
    {
        this.xCoordinate = x;
        this.yCoordinate = y;
        this.theta = theta;
    }

    public void setInitialRobotState(double initialEncoderLeft, double initialEncoderRight, double initialGyroAngle)
    {
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


}
