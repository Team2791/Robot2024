package frc.robotkt.swerve

import frc.robotkt.constants.kTau
import kotlin.math.abs

/**
 * Step a value towards a target with a step size
 * @param current The current value
 * @param target The target value
 * @param step The step size
 * @return The new value
 */
fun stepTowards(current: Double, target: Double, step: Double): Double {
    val diff = target - current
    return when {
        abs(diff) < step -> target
        diff > 0 -> current + step
        else -> current - step
    }
}

/**
 * Steps an angular value towards a target with a step size. Uses radians
 * @param current The current angular value
 * @param target The target angular value
 * @param step The step size
 * @return The new angular value, 0 <= value < 2π
 */
fun stepTowardsAngle(current: Double, target: Double, step: Double): Double {
    val diff = normalizeAngle(target - current)
    val target = normalizeAngle(target)

    return when {
        diff < step -> target
        diff > Math.PI -> normalizeAngle(current - step)
        else -> normalizeAngle(current + step)
    }
}

/**
 * Unsigned minimum difference between two angles in radians, calculating across the 0/2π boundary
 * @param a The first angle
 * @param b The second angle
 * @return The minimum difference between the two angles
 */
fun angleDifference(a: Double, b: Double): Double {
    val diff = normalizeAngle(a - b)
    return if (diff > Math.PI) kTau - diff else diff
}


/**
 * Turns an angle in radians into the range 0 <= angle < 2π
 * @param angle The angle in radians. Can be negative
 * @return The angle in the range 0 <= angle < 2π
 */
fun normalizeAngle(angle: Double): Double {
    var mod = angle % kTau
    if (mod < 0) mod += kTau

    return mod
}