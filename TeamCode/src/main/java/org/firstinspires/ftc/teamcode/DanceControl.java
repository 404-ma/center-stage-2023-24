package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Helper.ClawMoves.PARAMS;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.Helper.Crane;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions.DeferredActionType;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.Launcher;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;

import java.util.List;
import java.util.Locale;


@TeleOp(name = "Dance Control", group = "Demo")
public class DanceControl extends LinearOpMode {
    private static final String version = "0.0";
    private boolean setReversed = false;
    private ClawMoves yclaw;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Dance Control");
        telemetry.addData("Version Number", version);
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        gamePadInputV2 gpIn1 = new gamePadInputV2(gamepad1);
        gamePadInputV2 gpIn2 = new gamePadInputV2(gamepad2);
        DrivetrainV2 drvTrain = new DrivetrainV2(hardwareMap);
        Conveyor conv = new Conveyor(hardwareMap);
        Crane crane = new Crane(hardwareMap);
        yclaw = new ClawMoves(hardwareMap);
        Launcher launch = new Launcher(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        launch.startPosition();
        telemetry.clear();

        boolean suplex = false;
        double speedMultiplier = 1;
        double lastSpeed = 1;
        boolean viperOverride = false;

        while (opModeIsActive()) {
            update_telemetry(gpIn1, gpIn2);

            gamePadInputV2.GameplayInputType inpType = gpIn1.WaitForGamepadInput(30);
            switch (inpType) {
                case JOYSTICK:
                    drvTrain.setDriveVectorFromJoystick(gamepad1.left_stick_x * (float) speedMultiplier,
                            gamepad1.right_stick_x * (float) speedMultiplier,
                            gamepad1.left_stick_y * (float) speedMultiplier, setReversed);
                    break;

                case BUTTON_A: //make him clap his claws
                    yclaw.MoveArm(0.30); //our own position
                    sleep(150);
                    yclaw.MoveFlip(0.39);
                    sleep(200);
                    for(int i=0; i<5; i++){
                        yclaw.closeGrip();
                        sleep(120);
                        yclaw.openGrip(); //make our own position
                        sleep(120);
                    }
                    break;


                case BUTTON_L_BUMPER:
                    suplex = !suplex;
                    if (!suplex) {
                        yclaw.PrepForPixel(true);
                    } else {
                        yclaw.SuplexPixel();
                    }
                    break;

                //TODO say no button
                case BUTTON_B://say no button
                    drvTrain.setDriveVector(0,0, .035);
                    sleep(2000);
                    drvTrain.setDriveVector(0,0, -.070);
                    sleep(2000);
                    drvTrain.setDriveVector(0,0, .035);
                    sleep(2000);
                    break;

                //TODO throw the pixel
                case BUTTON_Y:
                    if (yclaw.tlmGripPosition != PARAMS.gripClosedPos) {
                        yclaw.MoveGrip(PARAMS.gripClosedPos);
                        DeferredActions.CreateDeferredAction(150, DeferredActionType.CLAW_ARM_UP);
                    } else {
                        yclaw.MoveArm(PARAMS.armUpPos);
                    }
                    DeferredActions.CreateDeferredAction(180, DeferredActionType.CLAW_FLIP_SUPLEX);
                    // Wait for Pixel over Bin
                    DeferredActions.CreateDeferredAction(400, DeferredActionType.CLAW_OPEN_GRIP_UP);
                    break;

                //TODO point at someone
                case BUTTON_X:
                    yclaw.MoveArm(PARAMS.armUpPos);
                    SystemClock.sleep(150);
                    yclaw.MoveFlip(PARAMS.flipSuplexPos);
                    // Wait for Pixel over Bin
                    SystemClock.sleep(675);
                    yclaw.MoveGrip(PARAMS.gripOpenPosTop);
                    sleep(200);
                    yclaw.MoveFlip(PARAMS.flipDownPos);
                    yclaw.closeGrip();

                    for(int i = 0; i < 3; i++){
                        yclaw.MoveArm(0.28);
                        sleep(500);
                        yclaw.MoveArm(0.29);
                        sleep(500);}
                    break;
            }

            // Deferred Actions
            ProcessDeferredActions();
        }
    }

    public void ProcessDeferredActions(){
        List<DeferredActionType> action = DeferredActions.GetReadyActions();

        for(DeferredActionType actionType: action){
            switch(actionType){
                case CLAW_FLIP_SUPLEX:
                    yclaw.MoveFlip(PARAMS.flipSuplexPos);
                    break;

                case CLAW_OPEN_GRIP_UP:
                    yclaw.MoveGrip(PARAMS.gripOpenPosTop);
                    break;

                case CLAW_OPEN_GRIP_DOWN:
                    yclaw.MoveGrip(PARAMS.gripOpenPos);
                    break;

                case CLAW_ARM_UP:
                    yclaw.MoveArm(PARAMS.armUpPos);
                    break;

                case CLAW_ARM_DOWN:
                    yclaw.MoveArm(PARAMS.armDownPos);
                    break;

                default:
                    telemetry.addLine("ERROR - Unsupported Deferred Action");
                    break;
            }
        }
    }

    private void update_telemetry(gamePadInputV2 gpi1, gamePadInputV2 gpi2) {
        telemetry.addLine("Gamepad #1");
        String inpTime1 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi1.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP1 Time", inpTime1);
        telemetry.addLine().addData("GP1 Cnt", gpi1.getTelemetry_InputCount());
        telemetry.addLine().addData("GP1 Input", gpi1.getTelemetry_InputLastType().toString());
        telemetry.addLine().addData("L Joy  X", "%6.3f", gamepad1.left_stick_x).addData("Y", "%6.3f", gamepad1.left_stick_y);
        telemetry.addLine().addData("R Joy  X", "%6.3f", gamepad1.right_stick_x).addData("Y", "%6.3f", gamepad1.right_stick_y);

        telemetry.addLine();
        telemetry.addLine("Gamepad #2");
        String inpTime2 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi2.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP2 Time", inpTime2);
        telemetry.addLine().addData("GP2 Cnt", gpi2.getTelemetry_InputCount());
        telemetry.addLine().addData("GP2 Input", gpi2.getTelemetry_InputLastType().toString());
        telemetry.addLine().addData("L Joy  X", "%6.3f", gamepad2.left_stick_x).addData("Y", "%6.3f", gamepad2.left_stick_y);
        telemetry.addLine().addData("R Joy  X", "%6.3f", gamepad2.right_stick_x).addData("Y", "%6.3f", gamepad2.right_stick_y);
        telemetry.addLine();
        telemetry.addLine("Deferred Actions");
        String actTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(DeferredActions.tlmLastActionTimestamp);
        telemetry.addLine().addData("Time", actTime);
        telemetry.addLine().addData("Action", DeferredActions.tlmLastAction.toString());

        telemetry.update();
    }
}
