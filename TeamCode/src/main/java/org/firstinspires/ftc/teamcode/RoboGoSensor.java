package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.TestAprilTags;
//import org.firstinspires.ftc.robotcore.TestAprilTags.initAprilTags;




import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Abika and Subo are amazing") //this is the file name in hub

@Disabled
public class RoboGoSensor extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    private static final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)

    private static final double FINAL_DISTANCE = 10; //this is how close the sensors should get to the target (cm)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    private static final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private static  final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    private static final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up     to 25% power at a 25 degree error. (0.25 / 25.0)

    private static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    //private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    //private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    //private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private gamePadInputV2 game1;
    private DrivetrainV2 game2;
    private TestAprilTags see = new TestAprilTags();

    @Override public void runOpMode()
    {

        game1 = new gamePadInputV2(gamepad1);
        game2 = new DrivetrainV2(hardwareMap);

        DistanceSensor distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        DistanceSensor distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");


        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process

        see.initAprilTag();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        int bNum = 0;
        boolean autoPilot = false;


        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            GameplayInputType iN = game1.WaitForGamepadInput(100);

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","Press B button to turn on\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("BearingDDDD","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Right Sensor Distance", distanceR.getDistance(DistanceUnit.CM));
                telemetry.addData("Left Sensor Distance", distanceL.getDistance(DistanceUnit.CM));

            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            //telemetry.addData("Test: ",GameplayInputType.RIGHT_TRIGGER_ON);
            telemetry.addData("Test1: ",iN);

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .


            if(iN == GameplayInputType.BUTTON_B){
                bNum++;
            }

            if(bNum % 2 == 1){
                autoPilot = true;
            }
            else if(bNum % 2 == 0){
                autoPilot = false;
            }

            if (iN == GameplayInputType.JOYSTICK) {
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  =  gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   =  gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                //game2.SetDriveVector(drive,strafe,turn);
            }

            else if(autoPilot & targetFound){
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = ( DESIRED_DISTANCE -  desiredTag.ftcPose.range);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;


                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            else{
                drive = 0;
                strafe = 0;
                turn = 0;
                telemetry.addData("Stationary", "Drive %0f, Strafe %0f, Turn %0f ", drive, strafe, turn);
            }

            double sensorEr = (distanceL.getDistance(DistanceUnit.CM) + distanceR.getDistance(DistanceUnit.CM) )/2;


            if(desiredTag.ftcPose.range == 12 && targetFound){

                double rangeESen = (FINAL_DISTANCE - sensorEr);
              // double headingError = desiredTag.ftcPose.bearing;
              //double yawError = desiredTag.ftcPose.yaw;



                drive = Range.clip(rangeESen * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
              //  turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
              // strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                //calculates rotational and distance power

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();
            game2.setDriveVector(drive,strafe,turn);

            //strafe = zero
            //drive = powerdistance
            //turn = powerrot
            //add it to setDriveVector

            // Apply desired axes motions to the drivetrain.

        }
    }




    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */


    /**
     * Initialize the AprilTag processor.
     */

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

