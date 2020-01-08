/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib.vision;

/**
 * This class is used to store information about a vision targets.  A target will include an offset, a distance, and an angle for locating the target
 */
public class VisionTarget 
{
    private double offCenter; //In meters
    private double distance; //In meters
    private double angle; //In radians

    //Constructors
    public VisionTarget()
    {
        offCenter = 0;
        distance = 0;
        angle = 0;
    }

    public VisionTarget(double metersOffCenter, double metersDistance, double angleInRadians)
    {
        offCenter = metersOffCenter;
        distance = metersDistance;
        angle = angleInRadians;
    }


    //Methods

    public double getOffCenter()
    {
        return offCenter;
    }

    public double getDistance()
    {
        return distance;
    }

    public double getAngleInDegrees()
    {
        return Math.toDegrees(angle);
    }

    public double getAngleInRadians()
    {
        return angle;
    }

    public void setOffCenter(double metersOffCenter)
    {
        offCenter = metersOffCenter;
    }

    public void setDistance(double metersDistance)
    {
        distance = metersDistance;
    }

    public void setAngleFromDegrees(double degreesAngle)
    {
        angle =  Math.toRadians(degreesAngle);
    }

    public void setAngleFromRadians(double radiansAngle)
    {
        angle = radiansAngle;
    }


}
