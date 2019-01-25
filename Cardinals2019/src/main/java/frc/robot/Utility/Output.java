/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;


public class Output 
{

    private double motorTopLeft;
    private double motorTopRight;
    private double motorBottomLeft;
    private double motorBottomRight;


    public Output()
    {
        motorTopLeft = 0;
        motorTopRight = 0;
        motorBottomLeft = 0;
        motorBottomRight = 0;

    }

    public Output(double leftValue, double rightValue)
    {
        motorTopLeft = leftValue;
        motorTopRight = rightValue;
        motorBottomLeft = 0;
        motorBottomRight = 0;
    }

    public Output(double topLeftValue, double topRightValue, double bottomLeftValue, double bottomRightValue)
    {
        motorTopLeft = topLeftValue;
        motorTopRight = topRightValue;
        motorBottomLeft = bottomLeftValue;
        motorBottomRight = bottomRightValue;
    }

    //Getter methods

    public double getLeftValue()
    {
        return motorTopLeft;
    }

    public double getRightValue()
    {
        return motorTopRight;
    }

    public double getTopLeftValue()
    {
        return motorTopLeft;
    }

    public double getTopRightValue()
    {
        return motorTopRight;
    }

    public double getBottomLeftValue()
    {
        return motorBottomLeft;
    }

    public double getBottomRightValue()
    {
        return motorBottomRight;
    }

    //Setter methods

    public void updateOutput(double leftValue, double rightValue)
    {
            motorTopLeft = handleLimits(leftValue);
            motorTopRight = handleLimits(rightValue); 
    }

    public void updateOutput(double topLeftValue, double topRightValue, double bottomLeftValue, double bottomRightValue)
    {
        motorTopLeft = handleLimits(topLeftValue);
        motorTopRight = handleLimits(topRightValue);
        motorBottomLeft = handleLimits(bottomLeftValue);
        motorBottomRight = handleLimits(bottomRightValue);
    }

    //Clear all values

    public void clearOutput()
    {
        motorTopLeft = 0;
        motorTopRight = 0;
        motorBottomLeft = 0;
        motorBottomRight = 0;
    }



    private double handleLimits(double input)
    {
        if(input > 1 )
        {
            return 1;
        }
        else if(input < -1)
        {
            return -1;
        }
        else
        {
            return input;
        }
    }
}
