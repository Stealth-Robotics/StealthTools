//----------------------------------------------------------------------------
//
//  $Workfile: StopWatch.java$
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
//Class Name: StopWatch
//
//Purpose:
//  Astop watch for timeing things
//
//----------------------------------------------------------------------------
public class StopWatch 
{
    //----------------------------------------------------------------------------
    //  Class Attributes 
    //----------------------------------------------------------------------------	
	long mLastTime = 0;
	int mWaitTime = 1000;  // 1000ms
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Constructor, with setting the time
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public StopWatch(int waitTime)
	{
		mWaitTime = waitTime;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Reset the wait time
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------	
	public void setTime(int waitTime)
	{
		mWaitTime = waitTime;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Has the stop watch expired
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------		
	public boolean isExpired()
	{
		if((System.currentTimeMillis()-mLastTime)>mWaitTime)
		{
			return true;
		}
		return false;
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Reset the stopwatch to 0 ms
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------		
	public void reset()
	{
		mLastTime = System.currentTimeMillis(); 
	}
	
    //--------------------------------------------------------------------
    // Purpose:
    //     Return the time left
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------			
	public long timeLeft()
	{
		return mWaitTime - (System.currentTimeMillis()-mLastTime);
	}
}
