package stealth_libraries;

public class StealthMath {
	
    /** @return 10% deadband */
	static public double DeadBand(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/** @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
	static public double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
	}
	
	

}
