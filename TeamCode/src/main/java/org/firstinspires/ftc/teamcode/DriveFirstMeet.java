package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;

import java.util.Locale;

@TeleOp(name= "DriveFirstMeet ", group ="Test")
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
        int power = 1;

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        gamePadInputV2 gpIn1 = new gamePadInputV2(gamepad1);
        gamePadInputV2 gpIn2 = new gamePadInputV2(gamepad2);
        DrivetrainV2 drvTrain = new DrivetrainV2(hardwareMap);
        update_telemetry(gpIn1, drvTrain);

        while (opModeIsActive()) {
            ++tlm_MainLoopCount;
            update_telemetry(gpIn1,gpIn2,drvTrain);
            gamePadInputV2.GameplayInputType inpType = gpIn1.WaitForGamepadInput(500);
            switch (inpType) {
                case LEFT_TRIGGER:
                    viperMotor.setPower(gamepad1.left_trigger * -1);
                    break;
                case RIGHT_TRIGGER:
                    viperMotor.setPower(gamepad1.right_trigger * 1);
                    break;
                case BUTTON_A:
                    conveyor.setPower(power);
                    break;
                case NONE:
                    conveyor.setPower(0);
                    break;
                case BUTTON_B:
                    power = -power;
                case JOYSTICK:
                    // Set Drivetrain Power
                    drvTrain.setDriveVectorFromJoystick(  gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_stick_y );
                    break;
            }
        }
    }


    private void update_telemetry(gamePadInputV2 gpi,gamePadInputV2 gpi,DrivetrainV2 drv) {
        telemetry.addLine().addData( "Main Loop Cnt", tlm_MainLoopCount);

        telemetry.addLine("Game-pad Input");
        telemetry.addLine().addData( "Input Ct", gpi.getTelemetry_InputCount());
        telemetry.addLine().addData( "Inp Last", gpi.getTelemetry_InputLastType().toString());
        String inpTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData( "Inp Time", inpTime);
        telemetry.addLine().addData( "L Joy  X", "%6.3f", gamepad1.left_stick_x).addData("Y", "%6.3f", gamepad1.left_stick_y);
        telemetry.addLine().addData( "R Joy  X", "%6.3f", gamepad1.right_stick_x).addData("Y", "%6.3f", gamepad1.right_stick_y);
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

