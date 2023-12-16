package org.firstinspires.ftc.teamcode.Helper;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.twoCamera;
public class DistanceSystem {


        public static DistanceSensor distanceR;
        public static DistanceSensor distanceL;

        public DistanceSystem(@NonNull HardwareMap hdwMap) {
            distanceR = hdwMap.get(DistanceSensor.class, "distanceR");
            distanceL = hdwMap.get(DistanceSensor.class, "distanceL");
        }

        public static double getDistance() {
            double distRCM = distanceR.getDistance(DistanceUnit.CM);
            double distLCM = distanceL.getDistance(DistanceUnit.CM);
            double avgdist = (distRCM + distLCM) / 2;




            return avgdist;
        }
    }



