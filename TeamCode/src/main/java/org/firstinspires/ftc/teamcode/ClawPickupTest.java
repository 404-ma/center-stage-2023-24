package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;


import java.util.Locale;

@Config
@TeleOp(name="Claw Pickup Test", group ="Hardware")
public class ClawPickupTest extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public double armStartPos = 0.5;
        public double armDownPos = 0.5;
        public double armStartFwd = 1;

        public double flipStartPos = 0.5;
        public double flipDownPos = 0.5;
        public double flipStartFwd = 1;

        public double gripStartPos = 0.5f;
        public double gripDownPos = 0.5f;
        public double gripStartFwd = 1;
    }

    public static Params PARAMS = new Params();

    // Internal Variables
    private gamePadInputV2 gpInput;
    private FtcDashboard dashboard;
    private Servo arm;
    private Servo flip;
    private Servo grip;

    private float armPosition = (float) PARAMS.armStartPos;
    private float armDownPosition = (float) PARAMS.armDownPos;
    private boolean armForward = (PARAMS.armStartFwd != 0);

    private float flipPosition = (float) PARAMS.flipStartPos;
    private float flipDownPosition = (float) PARAMS.flipDownPos;
    private boolean flipForward = (PARAMS.flipStartFwd != 0);

    private float gripDownPosition = (float) PARAMS.gripDownPos;
    private float gripPosition = (float) PARAMS.gripStartPos;
    private boolean gripForward = (PARAMS.gripStartFwd != 0);

    @Override
    public void runOpMode() {
        boolean good_init = initialize();
        waitForStart();
        if (isStopRequested() || !good_init)
            return;
        telemetry.clear();


        while (opModeIsActive()) {
            update_telemetry();

            gamePadInputV2.GameplayInputType inpType = gpInput.WaitForGamepadInput(100);
            switch (inpType) {
                case DPAD_UP:
                    arm.setPosition(armPosition);
                    break;

                case DPAD_DOWN:
                    arm.setPosition(armDownPosition);
                    break;

                case BUTTON_R_BUMPER:
                    flip.setPosition(flipPosition);

                    break;

                case BUTTON_L_BUMPER:
                    flip.setPosition(flipDownPosition);
                    break;


                case BUTTON_A:
                    grip.setPosition(gripDownPosition);
                    break;

                case BUTTON_B:
                    grip.setPosition(gripPosition);
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
            dashboard = FtcDashboard.getInstance();
            dashboard.clearTelemetry();
            gpInput = new gamePadInputV2(gamepad1);
            arm = hardwareMap.servo.get("ArmServo");
            flip = hardwareMap.servo.get("FlipServo");
            grip = hardwareMap.servo.get("ClawServo");
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
        telemetry.addLine("TODO: Add Data Here...");
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Position",  arm.getPosition());
        packet.put("Flip Position",  flip.getPosition());
        packet.put("Grip Position",  grip.getPosition());
        dashboard.sendTelemetryPacket(packet);
    }

}
