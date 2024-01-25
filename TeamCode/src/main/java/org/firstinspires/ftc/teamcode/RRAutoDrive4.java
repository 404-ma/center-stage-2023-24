package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.ClawPickupTest;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType;

@Config
@Autonomous(name = "RR Auto Drive 4 - Box", group = "RoadRunner")
public class RRAutoDrive4 extends LinearOpMode {
    public static class Params {
        public double endX = 24;
        public double endY = 80;
        public double endHeading = 90;


        private gamePadInputV2 gpInput;
        private FtcDashboard dashboard;

        //Parameter for Claw
        public static ClawPickupTest.Params PARAMS = new ClawPickupTest.Params();
        // Internal Variables
        private boolean tlmArmForward = false;
        private double tlmArmPosition = 0;
        private boolean tlmGripForward = false;
        private double tlmGripPosition = 0;
        private boolean tlmFlipForward = false;
        private double tlmFlipPosition = 0;
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
    private FtcDashboard dashboard;
    private MecanumDrive drive;

    @Override
    public void runOpMode() {

       Servo arm = hardwareMap.servo.get("ArmServo");
       Servo flip = hardwareMap.servo.get("FlipServo");
       Servo grip = hardwareMap.servo.get("ClawServo");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive 2 - Box");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        // Right Spike Mark
        Start(arm, flip, grip);

        Action moveTwo = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(18, 3), Math.toRadians(30))
                .build();
        Actions.runBlocking(moveTwo);

        PlacePixel(arm, flip, grip);

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo( new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveBack);

        Retract(arm, flip, grip);

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .build();
        Actions.runBlocking(moveBar);

        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo( new Vector2d(PARAMS.endX,  PARAMS.endY), Math.toRadians(PARAMS.endHeading))
                .build();
        Actions.runBlocking(moveOne);


    }

    public void PlacePixel(Servo arm, Servo flip, Servo grip){
        // Reset Claw to Down and Open
        arm.setPosition(PARAMS.armDownPos);
        sleep(100);  // let Arm Move Away from Conveyor
        flip.setPosition(PARAMS.flipDownPos);
        sleep(500);
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void Retract(Servo arm, Servo flip, Servo grip){
        arm.setPosition(PARAMS.armUpPos);
        sleep(100);
        flip.setPosition(PARAMS.flipSuplexPos);
        // Wait for Suplex to Finish
        grip.setPosition(PARAMS.gripOpenPos);
    }
    public void Start( Servo arm,Servo flip, Servo grip){
        grip.setPosition(PARAMS.gripClosedPos);
        arm.setPosition(PARAMS.armUpPos);
        flip.setPosition(PARAMS.flipSuplexPos);

    }



}





