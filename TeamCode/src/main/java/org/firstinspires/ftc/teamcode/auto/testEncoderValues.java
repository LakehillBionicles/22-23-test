package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled

public class testEncoderValues extends AutoBase {

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){

        //test color values


          //  senseColorsPort();



            encoderDrive(0.5, 24, 24, 20);


            sleep(1000);

            telemetry.addData("fpd", robot.fpd.getCurrentPosition());
            telemetry.addData("fsd", robot.fsd.getCurrentPosition());
            telemetry.addData("bpd", robot.bpd.getCurrentPosition());
            telemetry.addData("bsd", robot.bsd.getCurrentPosition());
            telemetry.addData("leftPost", robot.leftEncoder.getCurrentPosition());
            telemetry.addData("rightPost", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("frontPost", robot.frontEncoder.getCurrentPosition());


            telemetry.update();


            sleep(10000);






            //stop();

        }
    }
}