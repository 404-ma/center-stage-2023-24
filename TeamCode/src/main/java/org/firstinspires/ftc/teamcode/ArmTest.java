package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;

@TeleOp
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Servo grip = hardwareMap.servo.get("grip");
        Servo drone = hardwareMap.servo.get("DroneServo");
        gamePadInputV2 gpIn = new gamePadInputV2(gamepad1);
        double position = 0;

        telemetry.addLine("Use DpadUp/Down for big changes and DpadLeft/Right for small changes");
        telemetry.addLine("Use left/right bumpers to reverse direction");
        telemetry.addData("position:", position);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gamePadInputV2.GameplayInputType inpType = gpIn.WaitForGamepadInput(500);
            switch (inpType) {
                //start position 0.7
                case BUTTON_B:
                    position = 0.7;
                    telemetry.addData("position:", position);
                    telemetry.update();
                    drone.setPosition(position);
                    break;
                //drone servo is set to 0.25
                case BUTTON_X:
                    position = 0.25;
                    telemetry.addData("position:", position);
                    telemetry.update();
                    drone.setPosition(position);
                    break;

                case DPAD_UP:
                    position += 0.1;
                    telemetry.addData("position:", position);
                    telemetry.update();
                    break;
                case DPAD_DOWN:
                    position -= 0.1;
                    telemetry.addData("position:", position);
                    telemetry.update();
                    break;
                case DPAD_RIGHT:
                    position += 0.001;
                    telemetry.addData("position:", position);
                    telemetry.update();
                    break;
                 case BUTTON_A:
                    drone.setPosition(position);
                    break;
                case BUTTON_L_BUMPER:
                    drone.setDirection(Servo.Direction.REVERSE);
                    telemetry.addLine("Servo Is Reversed");
                    telemetry.addData("position:", position);
                    telemetry.update();
                    break;
                case BUTTON_R_BUMPER:
                    drone.setDirection(Servo.Direction.FORWARD);
                    telemetry.addLine("Servo Is Forwarded");
                    telemetry.addData("position:", position);
                    telemetry.update();
                    break;


            }
        }
    }
}