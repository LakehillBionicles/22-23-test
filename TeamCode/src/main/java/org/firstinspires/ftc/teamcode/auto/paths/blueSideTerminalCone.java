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


        while (opModeIsActive()){

            sideways(0.4, -14,-14, 3); //move to cone
            senseColorsPort(); //sense colors and store variable for later
            sleep(1000); //wait to make sure it sees colors
            sideways(0.4, -16, -16, 3); //cont sideways
            encoderDrive(0.4, -20, -20, 3); //back up to pole
            sideways(0.4, -10, -10, 3); //sideways so in front of pole
            armLift(1.0, 33, 7); //arm up -- double check inches here
            armLift(-1.0, 33, 7); //arm down -- double check inches here
            handDrop(); //open hand
            sleep(1000); //small wait to make sure hand has dropped -- we might be able to delete this part
            sideways(0.4, 10, 10, 3); //sideways away from pole
            encoderDrive(0.4, 20, 20, 3); //go back to center tiles where we started

            //parking method - deleted sideways strafe just b/c I don't think we need it

            if(senseColorsPort().equals( "red")){
                encoderDrive(0.4,-15, -15,3);
            }else if(senseColorsPort().equals("blue")){
                encoderDrive(0.4, 15,15,3);
            }else{
                //already in correct parking spot here??
            }

            stop();
        }
    }




}
