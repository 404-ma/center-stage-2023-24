package org.firstinspires.ftc.teamcode.helper;


/**
 * TargetPose represents the target position in space, relative to the robots viewpoint.
 * Note: These reference point for measurements is presumed to be the centerline of the sensor
 * measurement to the target.  There may be subtle variations in sensors but these values should
 * represent a reasonable approximation of distance.
 */
public class TargetPose {
    /**
     * Range (inches), (Distance), Average of the Sensor distance to the target, as measured along the X-Y plane (across the ground).
     */
    public double range;

    /**
     * Bearing (degrees), or Horizontal Angle, from the center-line of the robot to the center of the target.
     * This angle is measured across the X-Y plane (across the ground).<BR>
     * A positive Bearing indicates that the target is to the Left of the robot's centerline viewpoint.
     */
    public double bearing;


    /**
     * Yaw (degrees),  Rotation of the target around the Z axis. A yaw value of zero (0) means robot
     * is directly in front of target.  A positive Yaw implies the target is rotated Counter-Clockwise
     * when viewed from above.  Meaning it is closer to the robot on the left of the viewpoint
     * centerline and farther away on the right.
     */
    public double yaw;


    public TargetPose(double range, double bearing, double yaw) {
        this.range = range;
        this.bearing = bearing;
        this.yaw = yaw;
    }
}