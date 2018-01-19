//----------------------------------------------------------------------------
//
//  $Workfile: Navigation.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.VisionBot.subsystems;

import java.math.*;
import stealth_libraries.StealthMath;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: Navigation
//
//Purpose:
//  Run Odometry to find where we are on the field
//
//----------------------------------------------------------------------------
public class Navigation {
	
    //----------------------------------------------------------------------------
    //  Class Attributes 
    //----------------------------------------------------------------------------
	static private double mCurrentLocationX = 0.0;
	static private double mCurrentLocationY = 0.0;
	static private double mCurrentLocationTheta = 0.0;
	
	static private double mRobotWheelWidth = 0.0;
	
	static private double mOldLeftEncoder = 0.0;
	static private double mOldRightEncoder = 0.0;

	private static final Navigation mInstance = new Navigation();
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Constructor
    //
    // Notes:
    //     This is a singleton class, no one can build it
    //--------------------------------------------------------------------	
	private Navigation()
	{
		
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Gets the one and only instance of this class
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
    public static Navigation getInstance(){
        return mInstance;
    }	
	
    //--------------------------------------------------------------------
    // Purpose:
    //     We need to know how wide the wheels are
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
    public void setRobotWidth(double width)
    {
    	mRobotWheelWidth = width;
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Sets the inital location and angle of the robot
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
    public void setPoseRad(double x,double y,double theta)
    {
    	mCurrentLocationX = x;
    	mCurrentLocationY = y;
    	mCurrentLocationTheta = theta;
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Sets the inital location and angle of the robot
    //
    // Notes:
    //     This allows us to set the angle in degrees
    //--------------------------------------------------------------------	
    public void setPoseDeg(double x,double y,double theta)
    {
    	mCurrentLocationX = x;
    	mCurrentLocationY = y;
    	mCurrentLocationTheta = StealthMath.DegToRad(theta);
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Resets everything
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
    public void reset()
    {
    	mCurrentLocationX = 0.0;
    	mCurrentLocationY = 0.0;
    	mCurrentLocationTheta = 0.0;
    	
    	mRobotWheelWidth = 26;
    	
    	mOldLeftEncoder = 0.0;
    	mOldRightEncoder = 0.0;
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Drive the robot around and find the current location
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public void setNewPose(double leftEncoder, double rightEncoder)
	{
		double deltaLeft = leftEncoder - mOldLeftEncoder;
		double deltaRight = rightEncoder - mOldRightEncoder;
		
		double inches = (deltaLeft + deltaRight) / 2.0;
		
		mCurrentLocationTheta += (deltaLeft - deltaRight) / mRobotWheelWidth;
		
		mCurrentLocationTheta = StealthMath.KeepAsCircle(mCurrentLocationTheta);
		
		mCurrentLocationY += inches * Math.cos(mCurrentLocationTheta); 
        mCurrentLocationX += inches * Math.sin(mCurrentLocationTheta); 
		
		mOldLeftEncoder = leftEncoder;
		mOldRightEncoder = rightEncoder;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the X of the robot
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double getX()
	{
		return mCurrentLocationX; 
	}

    //--------------------------------------------------------------------
    // Purpose:
    //     Return the Y of the robot
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double getY()
	{
		return mCurrentLocationY; 
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the Theta of the robot
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double getTheta()
	{
		return mCurrentLocationTheta; 
	}
	
  // --------------------------------------------------------------------
  // Purpose:
  // Return the Theta of the robot in degrees
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
	public double getThetaDeg()
	{
		return StealthMath.RadToDeg(mCurrentLocationTheta); 
	}
}
