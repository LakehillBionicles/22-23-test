package org.firstinspires.ftc.teamcode.auto.oldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.auto.AutoBase;

@Autonomous
@Disabled
//@Disabled

public class redSideRedTerminalParking extends AutoBase {
    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        //CHECK TO MAKE SURE THESE NUMBERS ARE CORRECT

        while (opModeIsActive()){



            sideways(0.5, 16,16,3);//port is negative and starboard is positive
            if(senseColorsStar().equals( "red")){
                sideways(0.4, 7.5,7.5, 3);
                encoderDrive(0.4,15, 15,3);
            }else if(senseColorsStar().equals("blue")){
                sideways(0.4, 7.5,7.5, 3);
                encoderDrive(0.4, -15,-15,3);
            }else{
                sideways(0.4, 7.5,7.5, 3);
            }









            stop();
        }
    }


}
