package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class twoCamera extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DistanceSensor distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        DistanceSensor distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE );


        if (isStopRequested()) return;
        waitForStart();
        while (opModeIsActive()) {


            double distRCM = distanceR.getDistance(DistanceUnit.CM);
            double distLCM = distanceL.getDistance(DistanceUnit.CM);
            double avgdist = (distRCM+distLCM)/2;
            double kp = 0.05;


            double leftError = (distLCM - avgdist);
            double rightError = (distRCM - avgdist);

            double leftMotorSpeed = -(leftError * kp);
            double rightMotorSpeed = -(rightError * kp);

            if (rightError > 0.2){
                frontLeft.setPower(leftMotorSpeed);
                backLeft.setPower(leftMotorSpeed);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
            }

            if (leftError > 0.2){
                frontRight.setPower(rightMotorSpeed);
                backRight.setPower(rightMotorSpeed);
            } else {
                frontRight.setPower(0);
                backRight.setPower(0);
            }





            telemetry.addData("Right Sensor Distance", distanceR.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Sensor Distance", distanceL.getDistance(DistanceUnit.CM));
            telemetry.addData("Average Distance", avgdist);
            telemetry.update();
        }

    }
}