package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO:  Disable

public class eversonxc extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Bump sensor test");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();
        
        //It seems that only bump0 on the hardware map can output data
        DigitalChannel x0 = hardwareMap.get(DigitalChannel.class, "bump0");
        DigitalChannel x1 = hardwareMap.get(DigitalChannel.class, "bump1");

        while(opModeIsActive()){
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
            telemetry.addData("bump 0  ", x0.getState());
            telemetry.addData("bump 1 ", x1.getState());
            telemetry.addData(">","Press Start to Launch");
            telemetry.update();
            sleep(500);
        }
    }
}
