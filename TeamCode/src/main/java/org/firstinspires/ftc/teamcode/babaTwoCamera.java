package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class babaTwoCamera extends LinearOpMode {
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
        double initError = 0;
        double initErrorR = 0;
        double errorSum = 0;
        double errorSumR = 0;

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            while(gamepad1.right_stick_x == 0){


                double distRCM = distanceR.getDistance(DistanceUnit.CM);
                double distLCM = distanceL.getDistance(DistanceUnit.CM);
                double kp = 0.0625;
                double kd = 0.0;
                //-0.2;
                double ki = 0;
                //-.00001;
                double kpD = 0.0625;

                double kdD =0;
                //  -0.2;
                double kiD = 0;
                //.00001

                double targetDist = 10;



                double reading = (distLCM + distRCM)/2;

                double newError2 = (reading - targetDist);
                double kalError = (newError2+initError)/2;
                double diffError2 = -newError2 + initError;
                errorSum = newError2 + errorSum;
                initError = newError2;
                if (errorSum > 60){
                    errorSum = 60;
                }
                if (errorSum < -60){
                    errorSum = -60;
                }
                double error2Speed = 0;
                if (newError2 > 0.5 || newError2 < -0.5);
                error2Speed = (newError2 * kpD)+(diffError2*kdD)+(errorSum*kiD);

                double error = (distLCM - distRCM); //if negative right distance is greater than left
                double newError = error;
                double kalErrorR = (newError+initErrorR)/2;
                double diffErrorR = -newError + initErrorR;
                errorSumR = newError + errorSumR;
                initErrorR = newError;
                if (errorSumR > 60){
                    errorSumR = 60;
                }
                if (errorSumR < -60){
                    errorSumR = -60;
                }
                double rotationalSpeed = 0;
                if (error > 0.5 || error < -0.5) {
                    rotationalSpeed = (newError * kp) + (diffErrorR * kd) + (errorSumR * ki);
                }


                frontLeft.setPower(-rotationalSpeed + error2Speed);
                backLeft.setPower(-rotationalSpeed + error2Speed);
                frontRight.setPower(rotationalSpeed + error2Speed);
                backRight.setPower(rotationalSpeed + error2Speed);
                //if (error > 0.2){

                //} else if (error < -0.2){
                   // frontLeft.setPower(rotationalSpeed);
                    //backLeft.setPower(rotationalSpeed);
                    //frontRight.setPower(-rotationalSpeed);
                    //backRight.setPower(-rotationalSpeed);
                //}

                telemetry.addData("Right Sensor Distance", distanceR.getDistance(DistanceUnit.CM));
                telemetry.addData("Left Sensor Distance", distanceL.getDistance(DistanceUnit.CM));
                telemetry.addData("Error", error);
                telemetry.addData("rotational Speed", rotationalSpeed);
                telemetry.addData("proportional", kalError);
                telemetry.addData("derivate", diffError2);
                telemetry.addData("integral", errorSum);
                telemetry.update();
            }

        }

    }
}