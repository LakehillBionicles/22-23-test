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


            //code right here for put cone places

            sideways(0.4, -14,-14, 3);
            senseColorsPort();
            sleep(1000);
            sideways(0.4, -16, -16, 3);
            encoderDrive(0.4, -20, -20, 3);
            sideways(0.4, -10, -10, 3);
            //raise arm
            handDrop();
            sleep(1000);
            sideways(0.4, 10, 10, 3);
            encoderDrive(0.4, 20, 20, 3);

            //parking method - deleted sideways strafe just b/c I don't think we need it

            if(senseColorsPort().equals( "red")){
                encoderDrive(0.4,-15, -15,3);
            }else if(senseColorsPort().equals("blue")){
                encoderDrive(0.4, 15,15,3);
            }else{

            }









            stop();
        }
    }




}
