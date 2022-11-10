package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoBase;

@Autonomous
//@Disabled

public class blueSideTerminalCone extends AutoBase {

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()){


            turn(0.2, 90, 90, 10);



            sideways(0.4, -14,-14, 3); //move to cone

            //senseColorsPort(); //sense colors and store variable for later
            if(senseColorsPort().equals( "red")){
                color = "red";
            }else if(senseColorsPort().equals("blue")){
                color = "blue";
            }
            sleep(1000); //wait to make sure it sees colors
            sideways(0.4, -24, -23, 3); //c// ont sideways
            encoderDrive(0.4, -17, -17, 3); //back up to pole
            sleep(1000);//port is negative and starboard is positive
            sideways(0.4, -14, -14, 3);//sideways so in front of pole
            armLift(1.0, 33, 7); //arm up -- double check inches here
            encoderDrive(0.1, 2, 2, 3);
            handDrop(); //open hand
            encoderDrive(0.4, 2, 2, 3);
            armLift(-1.0, 33, 7); //arm down -- double check inches here
            sleep(1000); //small wait to make sure hand has dropped -- we might be able to delete this part
            sideways(0.4, 10, 10, 3); //sideways away from pole
            encoderDrive(0.4, 20, 20, 3); //go back to center tiles where we started

            //parking method - deleted sideways strafe just b/c I don't think we need it

            if(color.equals("red")){
                encoderDrive(0.4,-15, -15,3);
            }else if(color.equals("blue")){
                encoderDrive(0.4, 15,15,3);
            }else{
                //already in correct parking spot here??
            }

            stop();
        }
    }




}
