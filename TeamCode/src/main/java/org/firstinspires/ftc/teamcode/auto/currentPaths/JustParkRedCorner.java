package org.firstinspires.ftc.teamcode.auto.currentPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.AutoBase;

@Autonomous
//@Disabled

public class JustParkRedCorner extends AutoBase {

    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()) {

            coordinateDrive(0, 19, 0, .1, .1, .1, 5, .5, .5);
            //strafe sideways to see cone

            if(senseColorsStar().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsStar().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();


            sleep(200);

            coordinateDrive(0, 8, 0, .2, .2, .2, 5, .5, .5);

            if(color.equals("red")){
                coordinateDrive(0, 0, 26, .2, .2, .2, 5, .5, .5);

            }else if(color.equals("blue")){
                coordinateDrive(0, 22, 0, .2, .2, .2, 5, .5, .5);

            }else{
                coordinateDrive(0, 0, -24, .2, .2, .2, 5, .5, .5);
            }




            stop();




        }
    }
}