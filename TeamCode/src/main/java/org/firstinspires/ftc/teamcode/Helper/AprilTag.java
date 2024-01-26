package org.firstinspires.ftc.teamcode.Helper;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag{
    private static final double DESIRED_DISTANCE = 12;
    private static final double SPEED_GAIN  =  0.02;
    private static  final double STRAFE_GAIN =  0.015;
    private static final double TURN_GAIN   =  0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE= 0.5;
    private static final double MAX_AUTO_TURN  = 0.3;

    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private gamePadInputV2 game1;
    private DrivetrainV2 game2;


}

