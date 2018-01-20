//----------------------------------------------------------------------------
//
//  $Workfile: Encoder.java$
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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//----------------------------------------------------------------------------
//  Class Declarations
//----------------------------------------------------------------------------
//
// Class Name: Encoder
// 
// Purpose:
//      File to help convert encoders
//
//----------------------------------------------------------------------------
public class Encoder {

    //----------------------------------------------------------------------------
    //  Class Constants 
    //----------------------------------------------------------------------------
	static final int kBadTalon = 100;
	
    //----------------------------------------------------------------------------
    //  Class Attributes 
    //----------------------------------------------------------------------------
	WPI_TalonSRX mTalon;
	double mTicksPerRev = 100.0;
	double mRevToInches = 1.0;
	
  // --------------------------------------------------------------------
  // Purpose:
  // Constructor
  //
  // Notes:
  // None.
  // --------------------------------------------------------------------
	public Encoder()
	{
		mTalon = new WPI_TalonSRX(kBadTalon);
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Constructor
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public Encoder(WPI_TalonSRX talon)
	{
		mTalon = talon;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Sets the number of ticks per revolution
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public void SetTicksPerRev(double ticksPerRev)
	{
		mTicksPerRev = ticksPerRev;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Sets the rev to inches
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public void SetRevToInches(double revToInches)
	{
		mRevToInches = revToInches;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Clear the encoder
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public void Clear()
	{
		mTalon.setSelectedSensorPosition(0, 0, 20);
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the raw ticks
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double GetRawTicks()
	{
		return mTalon.getSelectedSensorPosition(0);
	}
	
  //--------------------------------------------------------------------
  // Purpose:
  //     Return the inches traveled
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------  
public double GetInches()
{
  if(0 != mTicksPerRev)
  {
    return mTalon.getSelectedSensorPosition(0) / mTicksPerRev * mRevToInches;
  }
  
  return 0.0;
}

//--------------------------------------------------------------------
// Purpose:
//     Return the feet traveled
//
// Notes:
//     None.
//--------------------------------------------------------------------  
public double GetFeet()
{
if(0 != mTicksPerRev)
{
  return GetInches()/12.0;
}

return 0.0;
}

    //--------------------------------------------------------------------
    // Purpose:
    //     Return raw velocity
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double GetRawVelocity()
	{
		return mTalon.getSelectedSensorVelocity(0);
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the inches per second
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public double GetVelocityInInches()
	{
		if(0 != mTicksPerRev)
		{
			return mTalon.getSelectedSensorVelocity(0) / mTicksPerRev * mRevToInches;
		}
		
		return 0.0;
	}
}
