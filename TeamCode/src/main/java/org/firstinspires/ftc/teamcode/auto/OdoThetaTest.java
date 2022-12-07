package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
//@Disabled

public class OdoThetaTest extends AutoBase {






    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            thetaTurn(90, .20, 3, 50000);

            sleep(50000);






            //stop();

        }
    }




    public void thetaTurn(double degrees, double targetPower, double tolerance, double timeoutS){

        robot.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        POWlocation = (robot.leftEncoder.getCurrentPosition());
        SOWlocation = (robot.rightEncoder.getCurrentPosition());


        targetTheta = (Math.toRadians(degrees) + (((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) * ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotTheta = (((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) * ODO_COUNTS_PER_INCH) / odoWheelGap);

        //newSPOWTarget = (((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2) + (int) (targetDistance));//convert target dist l8r
        //SPOWlocation = ((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2);

        resetRuntime();




        while(runtime.seconds() < timeoutS){
            while((Math.abs(robotTheta)) < ((Math.abs(targetTheta)) - Math.toRadians(tolerance))){
               /* POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());
                robotTheta = (((POWlocation + SOWlocation) * odoInches) / odoWheelGap);*/ //updates the encoder's position inside the loop

                telemetry.addData("are we there yet?", "no");
                telemetry.addData("targetTheta:", targetTheta);
                telemetry.addData("robot Theta: ", robotTheta);
                telemetry.addData("theta function:", (((POWlocation + SOWlocation) * ODO_COUNTS_PER_INCH) / odoWheelGap));

                telemetry.update();

                robot.fpd.setPower(-(targetPower));
                robot.bpd.setPower(-(targetPower));
                robot.fsd.setPower(targetPower);
                robot.bsd.setPower(targetPower);

               /* POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());*/
                robotTheta = (((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) * ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop


            }

            //SPOWlocation = ((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2);

            while ((Math.abs(robotTheta)) > ((Math.abs(targetTheta)) + Math.toRadians(tolerance))){
                /*POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());*/
                robotTheta = (((POWlocation + SOWlocation) * ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop

                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("targetTheta:", targetTheta);
                telemetry.addData("robot Theta: ", robotTheta);
                telemetry.addData("theta function:", (((POWlocation + SOWlocation) * ODO_COUNTS_PER_INCH) / odoWheelGap));
                telemetry.update();

                robot.fpd.setPower(targetPower);
                robot.bpd.setPower(targetPower);
                robot.fsd.setPower(-(targetPower));
                robot.bsd.setPower(-(targetPower));

               /* POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());*/
                robotTheta = (((robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) * ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop


            }

            while (((Math.abs(robotTheta)) > ((Math.abs(targetTheta)) - Math.toRadians(tolerance))) && ((Math.abs(robotTheta)) < ((Math.abs(targetTheta)) + Math.toRadians(tolerance)))){
                /*POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());
                robotTheta = (((POWlocation + SOWlocation) * odoInches) / odoWheelGap); *///updates the encoder's position inside the loop

                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("targetTheta:", targetTheta);
                telemetry.addData("robot Theta: ", robotTheta);
                telemetry.addData("theta function:", (((POWlocation + SOWlocation) * ODO_COUNTS_PER_INCH) / odoWheelGap));
                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

               /* POWlocation = (robot.leftEncoder.getCurrentPosition());
                SOWlocation = (robot.rightEncoder.getCurrentPosition());*/
                robotTheta = (((robot.leftEncoder.getCurrentPosition()+ robot.rightEncoder.getCurrentPosition()) * ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop
            }



        }

    }


}
