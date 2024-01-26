package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.gamePadInput;
import org.firstinspires.ftc.teamcode.Helper.gamePadInput.GameplayInputType;



@Disabled
public class twoSensor extends LinearOpMode {
    double kP;
    double kI;
    double kF;
    double kD;
    private gamePadInput game1;


    public void runOpMode() {

        DistanceSensor distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        DistanceSensor distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        game1 = new gamePadInput(gamepad1);

        GameplayInputType iN = game1.WaitForGamepadInput(100);

        //Error = (Target - current position) = 6-18 = 12

        // Creates a PIDFController with gains kP, kI, kD, and kF

        PIDFController pidf;
        pidf = new PIDFController(kP, kI, kD, kF);
        /*
         * Here are the constructors for the other controllers
         */
        PIDController pid = new PIDController(kP, kI, kD);
        PDController pd = new PDController(kP, kD);
        PController p = new PController(kP);
        // set our gains to some value
        pidf.setP(0.37);
        pidf.setI(0.05);
        pidf.setD(1.02);

// get our gain constants
        kP = pidf.getP();
        kI = pidf.getI();
        kD = pidf.getD();

// set all gains
        pidf.setPIDF(kP, kI, kD, 0.7);

        // get all gain coefficients
        double[] coeffs = pidf.getCoefficients();
        kP = coeffs[0];
        kI = coeffs[1];
        kD = coeffs[2];
        kF = coeffs[3];


        int yNum = 0;
        boolean autoPilot = false;

        if (iN == GameplayInputType.BUTTON_Y) {

            yNum++;
        }

        if (yNum % 2 == 1) {
            autoPilot = true;
        } else if (yNum % 2 == 0) {
            autoPilot = false;

        }
        double distRCM = distanceR.getDistance(DistanceUnit.CM);
        double distLCM = distanceL.getDistance(DistanceUnit.CM);
    }
}
