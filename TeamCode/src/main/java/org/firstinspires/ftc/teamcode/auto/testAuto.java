package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.AutoBase;


@Autonomous
//@Disabled

//test to see if everything works

public class testAuto extends AutoBase {


    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()) {



            //robot.fpd.setPower(1.0);
            //robot.fsd.setPower(1.0);
            //robot.bsd.setPower(1.0);
            //robot.bpd.setPower(1.0);

            driveUntilDist(0.5, 10.0);

            telemetry.addData("distance", robot.distSensor.getDistance(DistanceUnit.CM));
            telemetry.update();



            //encoderDrive(1.0, 24.0, 24.0, 2.0);
            //sideways(1.0, 12.0, 12.0, 2.0);

            //telemetry.addData("work", "good");
            //telemetry.update();
           stop();

        }
    }
}
