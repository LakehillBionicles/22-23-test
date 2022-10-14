package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
//@Disabled

public class blueSideBlueTerminal extends AutoBase{
    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){


            sideways(1.0, -23.0, -21.0, 3.0); //increase front in by 2 to account for drift

            //vision and store as variable for later

            sideways(1.0, -38.0, -36.0, 3.0); //line up w/ cones

            //haven't checked measurements or to see if works here down

            //driveUntilDist(1.0, 4.5, 3.0); //guestimating on distance right now

            //close hand

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
