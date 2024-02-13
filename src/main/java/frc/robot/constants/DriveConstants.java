package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    /**
     * meters per second
     */
    public static final double MAX_SPEED = 3;

    /**
     * radians per second
     */
    public static final double MAX_SPEED_ANGULAR = 2 * Math.PI;

    /**
     * radians per second
     */
    public static final double SLEW_DIRECTIONAL = 1.2;

    /**
     * percent per second (1 = 100%)
     */
    public static final double SLEW_MAGNITUDE = 1.8;

    /**
     * percent per second (1 = 100%)
     */
    public static final double SLEW_ROTATIONAL = 2.0;

    /**
     * meters
     */
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.5);

    /**
     * meters
     */
    public static final double WHEEL_BASE = TRACK_WIDTH;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    /**
     * Angular offsets of the modules relative to the chassis in radians
     */
    public static class AngularOffset {
        public static final double FRONT_LEFT = -Math.PI / 2;
        public static final double FRONT_RIGHT = 0;
        public static final double REAR_LEFT = Math.PI;
        public static final double REAR_RIGHT = Math.PI / 2;
    }

    public static class CANId {
        public static class Driving {
            public static final int FRONT_LEFT = 10;
            public static final int REAR_LEFT = 30;
            public static final int FRONT_RIGHT = 20;
            public static final int REAR_RIGHT = 40;
        }

        public static class Turning {
            public static final int FRONT_LEFT = 15;
            public static final int REAR_LEFT = 35;
            public static final int FRONT_RIGHT = 25;
            public static final int REAR_RIGHT = 45;
        }
    }

    public static final boolean GYRO_REVERSED = false;
}
