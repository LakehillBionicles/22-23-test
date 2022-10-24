package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
//@Disabled

public class blueSideBlueTerminal extends AutoBase{
    public void runOpMode(){
        super.runOpMode();
        startVision();
        startUp();
        waitForStart();

        while (opModeIsActive()){
            if(tagOfInterest == null || tagOfInterest.id == LEFT){
                telemetry.addData("Direction","Left");
            }else if(tagOfInterest.id == MIDDLE){
                telemetry.addData("Direction","Middle");
                //trajectory
            }else{
                telemetry.addData("Direction","Right");
                //trajectory
            }

        sideways(1.0, -23.0, -21.0, 3.0); //increase front in by 2 to account for drift

            //vision and store as variable for later

            //YOU MESSED THIS UP -- DEAL WITH PRELOADED CONE FIRST THEN CREATE TOUCH SENSOR LOOP

            sideways(1.0, -38.0, -36.0, 3.0); //line up w/ cones



            //haven't checked measurements or to see if works here down

            driveUntilDist(0.5, 7.0); //guestimating on distance right now

            telemetry.addData("did this work","yes");            //close hand
            telemetry.update();

            sleep(100000000);

            encoderDrive(1.0, -36.0, -36.0, 3.0); //back up until lined up with poles

            sideways(1.0, -8.0, -8.0, 3.0); //line up with pole

            //driveUntilDist(1.0, 4.5, 3.0); //drive until pole in grasp

            //method for raising hand and putting cone on and lowering hand

            encoderDrive(1.0, -5.0, -5.0, 3.0); //back out

            sideways(1.0, 8.0, 8.0, 3.0); //realign with cones

            //driveUntilDist(1.0, 4.5, 3.0); //drive until cones

            //repeat function once numbers are finalized... very tricky

            //parking function




            stop();
        }
    }
}
