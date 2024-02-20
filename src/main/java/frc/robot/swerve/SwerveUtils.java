package frc.robot.swerve;

public class SwerveUtils {

	/**
	 * Steps a value towards a target with a specified step size.
	 * @param current The current or starting value.  Can be positive or negative.
	 * @param target The target value the algorithm will step towards.  Can be positive or negative.
	 * @param step The maximum step size that can be taken.
	 * @return The new value for {@code _current} after performing the specified step towards the specified target.
	 */
	public static double StepTowards(double current, double target, double step) {
		if (Math.abs(current - target) <= step) {
			return target;
		} else if (target < current) {
			return current - step;
		} else {
			return current + step;
		}
	}

	/**
	 * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
	 * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
	 * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
	 * @param step The maximum step size that can be taken (in radians).
	 * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
	 * This value will always lie in the range 0 to 2*PI (exclusive).
	 */
	public static double StepTowardsCircular(double current, double target, double step) {
		current = WrapAngle(current);
		target = WrapAngle(target);

		double stepDirection = Math.signum(target - current);
		double difference = Math.abs(current - target);

		if (difference <= step) {
			return target;
		} else if (difference > Math.PI) {
			if (current + 2 * Math.PI - target < step || target + 2 * Math.PI - current < step) {
				return target;
			} else {
				return WrapAngle(current - stepDirection * step); //this will handle wrapping gracefully
			}

		} else {
			return current + stepDirection * step;
		}
	}

	/**
	 * Finds the (unsigned) minimum difference between two angles including calculating across 0.
	 * @param _angleA An angle (in radians).
	 * @param _angleB An angle (in radians).
	 * @return The (unsigned) minimum difference between the two angles (in radians).
	 */
	public static double AngleDifference(double _angleA, double _angleB) {
		double difference = Math.abs(_angleA - _angleB);
		return difference > Math.PI
		    ? (2 * Math.PI) - difference
		    : difference;
	}

	/**
	 * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
	 * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
	 * @return An angle (in radians) from 0 and 2*PI (exclusive).
	 */
	public static double WrapAngle(double _angle) {
		double twoPi = 2 * Math.PI;

		if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
			return 0.0;
		} else if (_angle > twoPi) {
			return _angle - twoPi * Math.floor(_angle / twoPi);
		} else if (_angle < 0.0) {
			return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
		} else {
			return _angle;
		}
	}
}