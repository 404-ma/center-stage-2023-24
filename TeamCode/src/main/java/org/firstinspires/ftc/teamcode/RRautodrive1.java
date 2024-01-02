package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;

@Autonomous (name = "RR auto drive 1", group = "RoadRunner")
public class RRautodrive1 extends LinearOpMode {

        @Override
        public void runOpMode(){
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
                telemetry.addLine("DistanceSystemtest");
                telemetry.addLine();
                telemetry.addData(">", "Press Start to Launch");
                telemetry.update();

                waitForStart();
                if (isStopRequested()) return;
                telemetry.clear();
                while (opModeIsActive()) {
                        double distance = DistanceSystem.getDistance();
                        double rangeError = (10 - distance);



                        


                }



        }
}







