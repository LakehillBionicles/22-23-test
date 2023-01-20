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

            coordinateDrive(0, 19, 0, .05, .05, .05, 4, .25, .25);
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

            coordinateDrive(0, 8, 0, .1, .1, .1, 4, .25, .25);

            if(color.equals("red")){
                coordinateDrive(0, 0, 26, .1, .1, .1, 4, .25, .25);

            }else if(color.equals("blue")){
                coordinateDrive(0, 22, 0, .1, .1, .1, 4, .25, .25);

            }else{
                coordinateDrive(0, 0, -24, .1, .1, .1, 4, .25, .25);
            }




            stop();




        }
    }
}