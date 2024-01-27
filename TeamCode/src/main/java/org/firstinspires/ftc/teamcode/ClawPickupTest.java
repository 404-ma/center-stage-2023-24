package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;



@Config
@TeleOp(name="Claw Pickup Test", group ="Hardware")
public class ClawPickupTest extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public double armForward = 1;
        public double armUpPos = 0.295;
        public double armDownPos = 0.2;

        public double flipForward = 1;
        public double flipDownPos = 0.54;
        public double flipSuplexPos = 0.395;

        public double gripForward = 1;
        public double gripOpenPos = 0.28;
        public double gripClosedPos = 0.10;
    }

    public static Params PARAMS = new Params();

    // Internal Variables
    private gamePadInputV2 gpInput;
    private FtcDashboard dashboard;
    private Servo arm;
    private Servo flip;
    private Servo grip;
    private ClawMoves whiteClaw;

    private boolean tlmArmForward = false;
    private double tlmArmPosition = 0;
    private boolean tlmGripForward = false;
    private double tlmGripPosition = 0;
    private boolean tlmFlipForward = false;
    private double tlmFlipPosition = 0;


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

            tlmArmForward = (PARAMS.armForward != 0);
            arm.setDirection(tlmArmForward ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);

            tlmGripForward = (PARAMS.gripForward != 0);
            grip.setDirection(tlmGripForward ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);

            tlmFlipForward = (PARAMS.flipForward != 0);
            flip.setDirection(tlmFlipForward ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);

            GameplayInputType inpType = gpInput.WaitForGamepadInput(100);
            switch (inpType) {
                case DPAD_UP:
                    ++level;
                    level = Math.max(level, 4);
                    whiteClaw.MoveLevel(level);
                    break;

                case DPAD_DOWN:
                    --level;
                    level = Math.max(level, 0);
                    whiteClaw.MoveLevel(level);
                    break;

                case DPAD_LEFT:
                case DPAD_RIGHT:
                    boolean close = (inpType == GameplayInputType.DPAD_RIGHT);
                    tlmGripPosition = close ? PARAMS.gripClosedPos : PARAMS.gripOpenPos;
                    grip.setPosition(tlmGripPosition);
                    break;

                case BUTTON_A:
                case BUTTON_Y:
                    boolean suplex = (inpType == GameplayInputType.BUTTON_A);
                    tlmFlipPosition = suplex ? PARAMS.flipSuplexPos : PARAMS.flipDownPos;
                    flip.setPosition(tlmFlipPosition);
                    break;

                case BUTTON_X:
                    whiteClaw.SuplexPixel();
                    break;

                case BUTTON_B:
                    whiteClaw.PrepForPixel();
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
            arm = hardwareMap.servo.get("ArmServo");
            flip = hardwareMap.servo.get("FlipServo");
            grip = hardwareMap.servo.get("ClawServo");
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
        telemetry.addLine().addData("Arm Dir", (tlmArmForward ? "Forward" : "Reverse") );
        telemetry.addLine().addData("Arm Pos", tlmArmPosition );
        telemetry.addLine().addData("Grip Dir", (tlmGripForward ? "Forward" : "Reverse") );
        telemetry.addLine().addData("Grip Pos", tlmGripPosition );
        telemetry.addLine().addData("Flip Dir", (tlmFlipForward ? "Forward" : "Reverse") );
        telemetry.addLine().addData("Flip Pos", tlmFlipPosition );

        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Position",  tlmArmPosition);
        packet.put("Flip Position",  flip.getPosition());
        packet.put("Grip Position",  tlmGripPosition);
        dashboard.sendTelemetryPacket(packet);
    }
}
