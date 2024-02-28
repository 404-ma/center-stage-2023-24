package org.firstinspires.ftc.teamcode.Helper;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
public class AutoCommon {


    public static void PlacePixel(boolean onlyOnePixel, MecanumDrive drive,
                                  ClawMoves claw, Conveyor conveyor) {

        // Shorter Conveyor Runtime if Only Depositing the Preloaded Pixel
        long runTimeMS = (onlyOnePixel ? 1200 : 2500);

        // Place Pixel on Backdrop
        claw.PrepForPixel(false);

        conveyor.moveViperToPosition(700);
        SystemClock.sleep(600);
        conveyor.moveConvForward();
        SystemClock.sleep(runTimeMS);
        conveyor.stopConv();
        conveyor.moveViperToPosition(0);
        SystemClock.sleep(1000);

        // Separate From Backdrop
        double forwardSeparation = drive.pose.position.y - 2;
        Action moveSeparateFromBackDrop = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .lineToY(forwardSeparation)
                .build();
        Actions.runBlocking(new ParallelAction(moveSeparateFromBackDrop, conveyor.RetractViperAction()));
    }


}
