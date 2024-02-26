package frc.robot.swerve;

import frc.robot.Constants;

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
			if (current + Constants.Circle.Tau - target < step || target + Constants.Circle.Tau - current < step) {
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
	 * @param a An angle (in radians).
	 * @param b An angle (in radians).
	 * @return The (unsigned) minimum difference between the two angles (in radians).
	 */
	public static double AngleDifference(double a, double b) {
		double difference = Math.abs(a - b);
		return difference > Math.PI
		    ? Constants.Circle.Tau - difference
		    : difference;
	}

	/**
	 * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
	 * @param angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
	 * @return An angle (in radians) from 0 and 2*PI (exclusive).
	 */
	public static double WrapAngle(double angle) {
		if (angle == Constants.Circle.Tau) {
			return 0.0;
		} else if (angle > Constants.Circle.Tau) {
			return angle - Constants.Circle.Tau * Math.floor(angle / Constants.Circle.Tau);
		} else if (angle < 0.0) {
			return angle + Constants.Circle.Tau * (Math.floor((-angle) / Constants.Circle.Tau) + 1);
		} else {
			return angle;
		}
	}
}