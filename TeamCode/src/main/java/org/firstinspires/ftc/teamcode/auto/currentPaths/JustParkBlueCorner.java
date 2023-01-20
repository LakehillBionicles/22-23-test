package org.firstinspires.ftc.teamcode.auto.currentPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.AutoBase;

@Autonomous
//@Disabled

public class JustParkBlueCorner extends AutoBase {

    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()) {

            coordinateDrive(2, -19, 0, .05, .05, .05, 5, .5, .5);
            //strafe sideways to see cone

            if(senseColorsPort().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsPort().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();


            sleep(200);

            /*coordinateDrive(0, -8, 0, .1, .1, .1, 4, .25, .25);

            if(color.equals("red")){
                coordinateDrive(0, 0, -21, .1, .1, .1, 4, .25, .25);

            }else if(color.equals("blue")){
                coordinateDrive(0, -22, 0, .1, .1, .1, 4, .25, .25);

            }else{
                coordinateDrive(0, 0, 26, .1, .1, .1, 4, .25, .25);
            }*/




            stop();




        }
    }
}