package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.AutoBase;

@Autonomous
//@Disabled

public class blueSideBlueTerminalParking extends AutoBase {
    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){

            //armLift(0.5, 3, 10);
            //sleep(10000);
            sideways(0.4, -14,-14,3);//port is negative and starboard is positive
            if(senseColorsPort().equals( "red")){
                sideways(0.4, -7.5,-7.5, 3);
                encoderDrive(0.4,-15, -15,3);
            }else if(senseColorsPort().equals("blue")){
                sideways(0.4, -7.5,-7.5, 3);
                encoderDrive(0.4, 15,15,3);
            }else{
                sideways(0.4, -7.5,-7.5, 3);
            }
            //correctSideways(1.0);
            //sleep(1000);//This will be vision
            //sideways(0.4, -25, -25, 3);
            //correctSideways(1.0);
            //encoderDrive(0.4,-26, -26, 3);

            //Vision








            stop();
        }
    }
}
