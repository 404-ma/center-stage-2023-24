package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Helper.ClawMoves.PARAMS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions.DeferredActionType;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;

import java.util.List;
import java.util.Locale;


// TODO: Lets Make a Group for Competition - Maybe Refactor to Different Name
@TeleOp(name = "DriveFirstMeet", group = "Competition!!")
public class DriveFirstMeet extends LinearOpMode {
    // TODO: Add @Config PARAMS values for all the speed multiplier Values
    // TODO: Add a Version Number Parameter

    private int tlm_MainLoopCount = 0;
    private boolean setReversed = false;
    private ClawMoves yclaw;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("TeleOp Drive Test");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();
        telemetry.addData("Version Number", "1:3");
        telemetry.update();

        // TODO:  Control viperMotor and conveyor servo via helper class.
        DcMotor viperMotor = hardwareMap.dcMotor.get("viperMotor");
        CRServo conveyor = hardwareMap.crservo.get("ConveyorServo");
        DcMotor craneMotor = hardwareMap.dcMotor.get("craneMotor");
        gamePadInputV2 gpIn1 = new gamePadInputV2(gamepad1);
        gamePadInputV2 gpIn2 = new gamePadInputV2(gamepad2);
        DrivetrainV2 drvTrain = new DrivetrainV2(hardwareMap);
        yclaw = new ClawMoves(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        boolean suplex = false;
        double speedMultiplier = 1;
        double lastSpeed = 1;

        while (opModeIsActive()) {
            ++tlm_MainLoopCount;
            update_telemetry(gpIn1, gpIn2, drvTrain);

            // TODO:  Add Function for Temporary Speed w/ Return to Previous Speed using Right Joystick Button
            gamePadInputV2.GameplayInputType inpType = gpIn1.WaitForGamepadInput(30);
            switch (inpType) {
                case RIGHT_STICK_BUTTON:
                    if (speedMultiplier != 1) {
                            lastSpeed = speedMultiplier;
                            speedMultiplier = 1;
                    } else if (lastSpeed != 1) {
                        speedMultiplier = lastSpeed;
                        lastSpeed = 1;
                    }


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
                    suplex = !suplex;
                    if (!suplex) {
                        yclaw.PrepForPixel(true);
                    } else {
                        yclaw.SuplexPixel();
                    }
                    break;

                case BUTTON_R_BUMPER:
                    if (yclaw.tlmGripPosition != PARAMS.gripOpenPos)
                        yclaw.openGrip();
                    else
                        yclaw.closeGrip();
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

                case BUTTON_BACK:
                    setReversed = !setReversed;
                    break;

                case JOYSTICK:
                    drvTrain.setDriveVectorFromJoystick(gamepad1.left_stick_x * (float) speedMultiplier,
                            gamepad1.right_stick_x * (float) speedMultiplier,
                            gamepad1.left_stick_y * (float) speedMultiplier, setReversed);
                    break;
            }

            gamePadInputV2.GameplayInputType inpType2 = gpIn2.WaitForGamepadInput(30);
            switch (inpType2) {
                // TODO: Is the Minimum Speed Really 0.75 - That make the Trigger range (0.75 - 1)
                case LEFT_TRIGGER:
                    double power = Math.min(gamepad2.left_trigger, 0.75);
                    conveyor.setPower(power);
                    break;
                case RIGHT_TRIGGER:
                    double powerBack = Math.min(gamepad2.right_trigger, 0.75);
                    conveyor.setPower(-powerBack);
                    break;
                case JOYSTICK:
                    viperMotor.setPower(gamepad2.right_stick_y * -0.5);
                    craneMotor.setPower(gamepad2.left_stick_y * -0.8);
                    break;
            }
            // Deferred Actions
            ProcessDeferredActions();
        }
    }

    // Deferred Actions
    public void ProcessDeferredActions(){
        List<DeferredActionType> action = DeferredActions.GetReadyActions();

        for(DeferredActionType actionType: action){
            switch(actionType){
                case CLAW_FLIP_SUPLEX:
                    yclaw.MoveFlip(PARAMS.flipSuplexPos);
                    break;

                case CLAW_OPEN_GRIP:
                    yclaw.MoveGrip(PARAMS.gripOpenPosTop);
                    break;

                case CLAW_FLIP_DOWN:
                    yclaw.MoveGrip(PARAMS.gripOpenPos);
                    yclaw.MoveFlip(PARAMS.flipDownPos);
                    break;

                case CLAW_ARM_SUPLEX:
                    yclaw.MoveArm(PARAMS.armUpPos);
                    break;
            }
        }
    }

    private void update_telemetry(gamePadInputV2 gpi1, gamePadInputV2 gpi2, DrivetrainV2 drv) {
        telemetry.addLine("Gamepad #1");
        String inpTime1 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi1.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP1 Time", inpTime1);
        telemetry.addLine().addData("GP1 Cnt", gpi1.getTelemetry_InputCount());
        telemetry.addLine().addData("GP1 Input", gpi1.getTelemetry_InputLastType().toString());
        telemetry.addLine().addData("L Joy  X", "%6.3f", gamepad1.left_stick_x).addData("Y", "%6.3f", gamepad1.left_stick_y);
        telemetry.addLine().addData("R Joy  X", "%6.3f", gamepad1.right_stick_x).addData("Y", "%6.3f", gamepad1.right_stick_y);

        telemetry.addLine("Gamepad #2");
        String inpTime2 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi2.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP2 Time", inpTime2);
        telemetry.addLine().addData("GP2 Cnt", gpi2.getTelemetry_InputCount());
        telemetry.addLine().addData("GP2 Input", gpi2.getTelemetry_InputLastType().toString());
        telemetry.addLine().addData("L Joy  X", "%6.3f", gamepad2.left_stick_x).addData("Y", "%6.3f", gamepad2.left_stick_y);
        telemetry.addLine().addData("R Joy  X", "%6.3f", gamepad2.right_stick_x).addData("Y", "%6.3f", gamepad2.right_stick_y);
        telemetry.update();
    }
}