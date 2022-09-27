package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoBase;


@Autonomous
//@Disabled


public class testAuto extends AutoBase {


    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()) {

            encoderDrive(1.0, 12.0, 12.0, 2.0);
            //sideways(1.0, 12.0, 12.0, 2.0);

            telemetry.addData("work", "good");
            telemetry.update();
        }
    }
}
