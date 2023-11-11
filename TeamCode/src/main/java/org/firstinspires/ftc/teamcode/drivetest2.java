package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInput;


import java.util.Locale;

@TeleOp(name= "Anu is awesome drive test ", group ="drivetest2")
public class drivetest2 extends LinearOpMode {
    private int tlm_MainLoopCount = 0;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Coach Test - TeleOp Drive Test");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        gamePadInput gpIn = new gamePadInput(gamepad1);
        DrivetrainV2 drvTrain = new DrivetrainV2(hardwareMap);
        update_telemetry(gpIn, drvTrain);

        while (opModeIsActive()) {
            ++tlm_MainLoopCount;
            update_telemetry(gpIn, drvTrain);
            gamePadInput.GameplayInputType inpType = gpIn.WaitForGamepadInput(500);
            switch (inpType) {
                case LEFT_TRIGGER_ON:
                    drvTrain.setBrakeStatus(true);
                case LEFT_TRIGGER_OFF:
                    // Brake (ON/OFF)
                    drvTrain.setBrakeStatus( false);
                    break;
                case JOYSTICK:
                    // Set Drivetrain Power
                    drvTrain.setDriveVectorFromJoystick(  gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_stick_y );
                    break;
            }
        }
    }


    private void update_telemetry(gamePadInput gpi, DrivetrainV2 drv) {
        telemetry.addLine().addData( "Main Loop Cnt", tlm_MainLoopCount);

        telemetry.addLine("Game-pad Input");
        telemetry.addLine().addData( "Input Ct", gpi.getTelemetry_InputCount());
        telemetry.addLine().addData( "Inp Last", gpi.getTelemetry_InputLastType().toString());
        String inpTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData( "Inp Time", inpTime);
        telemetry.addLine().addData( "L Joy  X", "%6.3f", gamepad1.left_stick_x).addData("Y", "%6.3f", gamepad1.left_stick_y);
        telemetry.addLine().addData( "R Joy  X", "%6.3f", gamepad1.right_stick_x).addData("Y", "%6.3f", gamepad1.right_stick_y);
        drv.getTelemetryLastPowerFrontLeft();
        telemetry.addLine("Drive Train");
        telemetry.addLine().addData( "Pwr  L F", drv.getTelemetryLastPowerFrontLeft());
        telemetry.addLine().addData( "Pwr  L B", drv.getTelemetryLastPowerBackLeft());
        telemetry.addLine().addData( "Pwr  R F", drv.getTelemetryLastPowerFrontRight());
        telemetry.addLine().addData( "Pwr  R B", drv.getTelemetryLastPowerBackRight());
        telemetry.addLine().addData( "Braking ", drv.getBrakeStatus());
        telemetry.addLine().addData( "Brake Ct", drv.getTelemetryBrakeCount());
        telemetry.addLine().addData( "BTimeout", drv.getTelemetryBrakeTimeoutCount());

        telemetry.update();  // Update clears screen in (Default) Auto Clear Mode
    }

}

