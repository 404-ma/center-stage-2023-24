package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;

import java.util.Locale;

@TeleOp(name = "DriveFirstMeet", group = "Test")
public class DriveFirstMeet extends LinearOpMode {
    private int tlm_MainLoopCount = 0;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("TeleOp Drive Test");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();
        DcMotor viperMotor = hardwareMap.dcMotor.get("viperMotor");
        CRServo conveyor = hardwareMap.crservo.get("conveyor");

        Servo arm = hardwareMap.servo.get("arm");
        Servo flip = hardwareMap.servo.get("flip");
        Servo grip = hardwareMap.servo.get("grip");

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        gamePadInputV2 gpIn1 = new gamePadInputV2(gamepad1);
        gamePadInputV2 gpIn2 = new gamePadInputV2(gamepad2);
        DrivetrainV2 drvTrain = new DrivetrainV2(hardwareMap);
        ClawMoves yclaw = new ClawMoves(hardwareMap);
        Conveyor cyr = new Conveyor(hardwareMap);
        update_telemetry(gpIn1, gpIn2, drvTrain);

        boolean test = false;
        double speedMultiplier = 1;

        while (opModeIsActive()) {
            ++tlm_MainLoopCount;
            update_telemetry(gpIn1, gpIn2, drvTrain);
            gamePadInputV2.GameplayInputType inpType = gpIn1.WaitForGamepadInput(30);
            switch (inpType) {
                case DPAD_UP:
                    yclaw.moveLevel(3);
                    break;
                case DPAD_DOWN:
                    yclaw.moveLevel(1);
                    break;
                case DPAD_RIGHT:
                    yclaw.moveLevel(4);
                    break;
                case DPAD_LEFT:
                    yclaw.moveLevel(2);
                    break;
                case BUTTON_L_BUMPER:
                    test = !test;
                    if (!test) {
                        yclaw.PrepForPixel();
                    } else {
                        yclaw.SuplexPixel();
                    }
                    break;
                case BUTTON_R_BUMPER:
                    test = !test;
                    if (!test) {
                        yclaw.openGrip();
                    } else {
                        yclaw.closeGrip();
                    }
                    break;
                case BUTTON_A:
                    speedMultiplier = 0.25;
                    break;
                case BUTTON_X:
                    speedMultiplier = 0.75;
                    break;
                case BUTTON_B:
                    speedMultiplier = 0.5;
                    break;
                case BUTTON_Y:
                    speedMultiplier = 1;
                    break;
                case RIGHT_TRIGGER:
                    viperMotor.setPower(gamepad1.right_trigger * 1);
                    break;
                case LEFT_TRIGGER:
                    double power = Math.min(gamepad1.left_trigger, 0.5);
                    conveyor.setPower(power);
                    break;
                case JOYSTICK:
                    drvTrain.setDriveVectorFromJoystick(gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_stick_y);
                    break;
            }

            gamePadInputV2.GameplayInputType inpType2 = gpIn2.WaitForGamepadInput(30);
            switch (inpType2) {
                case RIGHT_TRIGGER:
                    double power = Math.min(gamepad1.left_trigger, 0.2);
                    conveyor.setPower(power);
                    break;
                case LEFT_TRIGGER:
                    double power2 = Math.min(gamepad1.left_trigger, -0.2);
                    conveyor.setPower(power2);
                    break;
                case JOYSTICK:
                    viperMotor.setPower(gamepad1.left_stick_y * 0.5);
                    //TODO;set gantryslide using rigth_stick_y
                    break;
            }
        }
    }

    private void update_telemetry(gamePadInputV2 gpi1, gamePadInputV2 gpi2, DrivetrainV2 drv) {
        telemetry.addLine().addData("Main Loop Cnt", tlm_MainLoopCount);

        telemetry.addLine("Game-pad Input");
        telemetry.addLine().addData("Input Ct", gpi1.getTelemetry_InputCount());
        telemetry.addLine().addData("Inp Last", gpi1.getTelemetry_InputLastType().toString());
        String inpTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi1.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("Inp Time", inpTime);
        telemetry.addLine().addData("L Joy  X", "%6.3f", gamepad1.left_stick_x).addData("Y", "%6.3f", gamepad1.left_stick_y);
        telemetry.addLine().addData("R Joy  X", "%6.3f", gamepad1.right_stick_x).addData("Y", "%6.3f", gamepad1.right_stick_y);
        telemetry.addLine("Drive Train");
        telemetry.addLine().addData("Pwr  L F", drv.getTelemetryLastPowerFrontLeft());
        telemetry.addLine().addData("Pwr  L B", drv.getTelemetryLastPowerBackLeft());
        telemetry.addLine().addData("Pwr  R F", drv.getTelemetryLastPowerFrontRight());
        telemetry.addLine().addData("Pwr  R B", drv.getTelemetryLastPowerBackRight());
        telemetry.addLine().addData("Braking ", drv.getBrakeStatus());
        telemetry.addLine().addData("Brake Ct", drv.getTelemetryBrakeCount());
        telemetry.addLine().addData("BTimeout", drv.getTelemetryBrakeTimeoutCount());

        telemetry.update();
    }
}