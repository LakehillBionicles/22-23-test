package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.AutoBase;


@Autonomous
//@Disabled


public class testAuto extends AutoBase {


    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("distance", robot.distSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            driveUntilDist(1.0, 10.0);

            //encoderDrive(1.0, 24.0, 24.0, 2.0);
            //sideways(1.0, 12.0, 12.0, 2.0);

            //telemetry.addData("work", "good");
            //telemetry.update();
            stop();

        }
    }
}
