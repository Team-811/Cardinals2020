/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.lib;

/**
 * This class is used to store information about some of the vision targets.  A target will include what is the target and the 
 * location of the target
 */
public class VisionTarget 
{
    private int id;
    private double offCenter; //In meters
    private double distance; //In meters

    //Constructors
    public VisionTarget()
    {
        id = 0;
        offCenter = 0;
        distance = 0;
    }

    public VisionTarget(int targetID, double metersOffCenter, double metersDistance)
    {
        id = targetID;
        offCenter = metersOffCenter;
        distance = metersDistance;
    }


    //Methods

    public int getID()
    {
        return id;
    }

    public double getOffCenter()
    {
        return offCenter;
    }

    public double getDistance()
    {
        return distance;
    }

    public void setID(int targetID)
    {
        id = targetID;
    }

    public void setOffCenter(double metersOffCenter)
    {
        offCenter = metersOffCenter;
    }

    public void setDistance(double metersDistance)
    {
        distance = metersDistance;
    }


}
