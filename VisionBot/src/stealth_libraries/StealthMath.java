//----------------------------------------------------------------------------
//
//  $Workfile: StealthMath.java$
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
package stealth_libraries;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: StealthMath
//
//Purpose:
//    Misc math functions
//
//----------------------------------------------------------------------------
public class StealthMath {
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the value with a dead band where it is zero
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	static public double DeadBand(double axisVal) {
		if (axisVal < -0.10)
		{
			return axisVal;
		}
		
		if (axisVal > +0.10)
		{
			return axisVal;
		}
		
		return 0;
	}

    //--------------------------------------------------------------------
    // Purpose:
    //     Trim the top and bottom off of a number
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	static public double Cap(double value, double peak) {
		if (value < -peak)
		{
			return -peak;
		}
		
		if (value > +peak)
		{
			return +peak;
		}
		return value;
	}
		
    //--------------------------------------------------------------------
    // Purpose:
    //     Keep angles in a circle.  if the angle goes over pi then 
	//     wrap it around to -pi
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	static public double KeepAsCircle(double value) {
		if(value < -Math.PI)
		{
			value = value+(Math.PI*2.0);
		}
		
		if (value > Math.PI)
		{
			value = value-(Math.PI*2.0);
		}
		return value;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Change the angle from rad to deg
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	static public double DegToRad(double angle)
	{
		return angle * Math.PI / 180.0;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Change the angle from deg to red
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	static public double RadToDeg(double angle)
	{
		return angle * 180.0 / Math.PI;
	}	
}
