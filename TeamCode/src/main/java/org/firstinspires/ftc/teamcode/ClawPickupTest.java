package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;


@TeleOp(name="Claw Pickup Test", group ="Hardware")
public class ClawPickupTest extends LinearOpMode {
    // Internal Variables
    private gamePadInputV2 gpInput;
    private FtcDashboard dashboard;
    private ClawMoves whiteClaw;


    @Override
    public void runOpMode() {
        boolean good_init = initialize();
        waitForStart();
        if (isStopRequested() || !good_init)
            return;
        telemetry.clear();

        int level = 1;
        while (opModeIsActive()) {
            update_telemetry();

            GameplayInputType inpType = gpInput.WaitForGamepadInput(100);
            switch (inpType) {
                case DPAD_UP:
                    ++level;
                    level = Math.max(level, 4);
                    whiteClaw.moveLevel(level );
                    break;

                case DPAD_DOWN:
                    --level;
                    level = Math.max(level, 0);
                    whiteClaw.moveLevel(level);
                    break;

                case DPAD_LEFT:
                    whiteClaw.MoveGrip(ClawMoves.PARAMS.gripOpenPos);
                    break;

                case DPAD_RIGHT:
                    whiteClaw.MoveGrip(ClawMoves.PARAMS.gripClosedPos);
                    break;

                case BUTTON_A:
                    whiteClaw.MoveFlip(ClawMoves.PARAMS.flipSuplexPos);
                    break;

                case BUTTON_Y:
                    whiteClaw.MoveFlip(ClawMoves.PARAMS.flipDownPos);
                    break;

                case BUTTON_X:
                    whiteClaw.SuplexPixel();
                    break;

                case BUTTON_B:
                    whiteClaw.PrepForPixel(false);
                    break;

                case BUTTON_L_BUMPER:
                    whiteClaw.AutonomousStart();
                    whiteClaw.PlacePixel();
                    break;

                case BUTTON_R_BUMPER:
                    whiteClaw.RetractArm();
                    break;
            }
        }
    }


    private boolean initialize() {
        boolean success = true;

        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Claw Pickup Test");
        telemetry.addLine();

        // Initialize Helpers
        try {
            gpInput = new gamePadInputV2(gamepad1);
            whiteClaw = new ClawMoves(hardwareMap);
            dashboard = FtcDashboard.getInstance();
            dashboard.clearTelemetry();
        } catch (Exception e) {
            success = false;
        }

        if (success) {
            telemetry.addLine("All Sensors Initialized");
            telemetry.addLine("");
            telemetry.addData(">", "Press Play to Start");
        } else {
            telemetry.addLine("");
            telemetry.addLine("*** INITIALIZATION FAILED ***");
        }
        telemetry.update();
        return (success);
    }


    private void update_telemetry() {
        telemetry.addLine("Servo Values");
        telemetry.addLine().addData("Arm Pos", whiteClaw.tlmArmPosition );
        telemetry.addLine().addData("Flip Pos", whiteClaw.tlmFlipPosition );
        telemetry.addLine().addData("Grip Pos", whiteClaw.tlmGripPosition );
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Position",  whiteClaw.tlmArmPosition);
        packet.put("Flip Position",  whiteClaw.tlmFlipPosition);
        packet.put("Grip Position",  whiteClaw.tlmGripPosition);
        dashboard.sendTelemetryPacket(packet);
    }
}
