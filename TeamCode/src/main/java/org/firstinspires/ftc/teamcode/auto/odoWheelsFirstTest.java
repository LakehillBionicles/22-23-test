package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
//@Disabled
public class odoWheelsFirstTest extends AutoBase{

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){

            encoderDrive(0.5, 24, 24, 20);

            sleep(1000);



            telemetry.addData("right ticks: ", getRightTicks());
            telemetry.addData("left ticks: ", getLeftTicks());
            telemetry.addData("front ticks: ", getFrontTicks());
            telemetry.update();


            sleep(2000000);



            //encoderDrive(0.5, 24, 24, 20);







            //stop();

        }
    }







}
