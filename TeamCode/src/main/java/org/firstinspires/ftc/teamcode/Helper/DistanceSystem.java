package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helper.TargetPose;

import java.util.Date;

public class DistanceSystem {
    public static DistanceSensor distanceR;
    public static DistanceSensor distanceL;

    // ----------------------------- Internal Class Variables
    private double filterDistLEstimate = 0;
    private double filterDistREstimate = 0;
    private double filterRotationEstimate = 0;

    // ----------------------------- Telemetry Data
    public Date tlm_LastCheckedTimestamp = new Date();
    public int tlm_CheckCount = 0;
    public double tlm_LeftDistance = 0;  // in Inches
    public double tlm_RightDistance = 0;  // in Inches
    public double tlm_CurrentRotation = 0;  // in Inches
    public TargetPose tlm_LastPose = new TargetPose(0,0,0);

    // ----------------------------- Constants
    private static final double MAXIMUM_SENSOR_RANGE = 16; // inches - Sensor Limit
    private static final double MAXIMUM_ROTATION_RANGE = 12; // inches - Sensor Confidence Limit for Range Prediction
    private static final double MAXIMUM_ROTATION_ANGLE = 45; // Degrees - Maximum YAW Angle for Target Pose

    private static final double SENSOR_SEPARATION = 4; // inches - Horizontal Distance Between

    /**
     *  The Kalman Filter Constant Represent Confidence in Prediction of Next State
     *  based on the expected difference from the last state, as limited by the
     *  actual potential for the robot values to change.
     *  see <a href="https://en.wikipedia.org/wiki/Kalman_filter">Kalman Filter</a> fpr more information
     */
    private static final double KALMAN_DISTANCE_FILTER = 0.8;  // Distance
    private static final double KALMAN_ROTATION_FILTER = 0.8;  // Rotation


    public DistanceSystem(@NonNull HardwareMap hdwMap) {
            distanceR = hdwMap.get(DistanceSensor.class, "distanceR");
            distanceL = hdwMap.get(DistanceSensor.class, "distanceL");
        }

    public TargetPose getTargetPose(boolean reset) {
        // Capture Telemetry
        tlm_LastCheckedTimestamp = new Date();
        ++tlm_CheckCount;
        tlm_LeftDistance = Math.min(distanceL.getDistance(DistanceUnit.INCH), MAXIMUM_SENSOR_RANGE);
        tlm_RightDistance = Math.min(distanceR.getDistance(DistanceUnit.INCH), MAXIMUM_SENSOR_RANGE);

        // Ignore Sensor Error Readings (0) If Other Sensor has a Good Reading
        if ((tlm_LeftDistance <= 0) && (tlm_RightDistance > 0))
            tlm_LeftDistance = tlm_RightDistance;
        if ((tlm_RightDistance <= 0) && (tlm_LeftDistance > 0))
            tlm_RightDistance = tlm_LeftDistance;

        // Compute Kalman Filter on Distance Sensors
        if (reset) {
            filterDistLEstimate = tlm_LeftDistance;
            filterDistREstimate = tlm_RightDistance;
        } else {
            // Ignore Sensor Reading Errors (0)
            filterDistLEstimate = (KALMAN_DISTANCE_FILTER * filterDistLEstimate)
                                    + ((1 - KALMAN_DISTANCE_FILTER) * tlm_LeftDistance);
            filterDistREstimate = (KALMAN_DISTANCE_FILTER * filterDistREstimate)
                                    + ((1 - KALMAN_DISTANCE_FILTER) * tlm_RightDistance);
        }

        double avgdist = (filterDistLEstimate + filterDistREstimate) / 2;
        TargetPose pose = new TargetPose(avgdist, 0, 0);

        // Check If Within Sensor Range Limits for Rotation
        if (avgdist <= MAXIMUM_ROTATION_RANGE) {
            // Calculate in Degrees
            tlm_CurrentRotation = Math.toDegrees(Math.atan( (filterDistREstimate - filterDistLEstimate) / SENSOR_SEPARATION));
            tlm_CurrentRotation = Range.clip(tlm_CurrentRotation, -MAXIMUM_ROTATION_ANGLE, MAXIMUM_ROTATION_ANGLE);

            // Compute Kalman Filter on Rotation
            if (reset)
                filterRotationEstimate = tlm_CurrentRotation;
            else
                filterRotationEstimate = (KALMAN_ROTATION_FILTER * filterRotationEstimate) + ((1 - KALMAN_ROTATION_FILTER) * tlm_CurrentRotation);

            pose.yaw = filterRotationEstimate;
        }

        tlm_LastPose = pose;
        return pose;
    }


    public static double getDistance() {
        double distRCM = distanceR.getDistance(DistanceUnit.CM);
        double distLCM = distanceL.getDistance(DistanceUnit.CM);
        double avgdist = (distRCM + distLCM) / 2;

        return avgdist;
    }
}

