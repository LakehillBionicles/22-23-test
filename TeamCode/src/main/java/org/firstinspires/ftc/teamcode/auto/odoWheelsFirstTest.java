package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import kotlin.ranges.URangesKt;

@Autonomous
//@Disabled
public class odoWheelsFirstTest extends AutoBase{

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("current pos:", robot.leftEncoder.getCurrentPosition());
            telemetry.update();

            sleep(1000);

            portSideDriveForward(100000, .25, .000001, .25);

            sleep(50000);


            /*encoderDrive(0.5, 24, 24, 20);

            sleep(1000);

                        telemetry.addData("right ticks: ", getRightTicks());
            telemetry.addData("left ticks: ", getLeftTicks());
            telemetry.addData("front ticks: ", getFrontTicks());
            telemetry.update();


            sleep(2000000); */



            //encoderDrive(0.5, 24, 24, 20);







            //stop();

        }
    }




    public void portSideDriveForward(double targetDistance, double targetPower, double tolerance, double overshootPower){

        newPOWTarget = robot.leftEncoder.getCurrentPosition() + (int) (targetDistance);//convert target dist l8r
        POWlocation = robot.leftEncoder.getCurrentPosition();


        while(POWlocation < (1-tolerance) * newPOWTarget){
           telemetry.addData("are we there yet?", "no");
           telemetry.addData("newPOWTarget:", newPOWTarget);
            telemetry.addData("POW location: ", POWlocation);
            telemetry.addData("leftEncoderPos:", robot.leftEncoder.getCurrentPosition());

            telemetry.update();

            /*robot.fpd.setPower(targetPower);
            robot.bpd.setPower(targetPower);
            robot.fsd.setPower(targetPower);
            robot.bsd.setPower(targetPower);*/

           POWlocation = robot.leftEncoder.getCurrentPosition(); //updates the encoder's position inside the loop


        }

        POWlocation = robot.leftEncoder.getCurrentPosition();
        while (POWlocation > (1+tolerance) * newPOWTarget){
            telemetry.addData("are we there yet?", "too far");
            telemetry.addData("newPOWTarget:", newPOWTarget);
            telemetry.addData("POW location: ", POWlocation);
            telemetry.addData("leftEncoderPos:", robot.leftEncoder.getCurrentPosition());

            telemetry.update();

            robot.fpd.setPower(-Math.abs(overshootPower));
            robot.bpd.setPower(-Math.abs(overshootPower));
            robot.fsd.setPower(-Math.abs(overshootPower));
            robot.bsd.setPower(-Math.abs(overshootPower));

            POWlocation = robot.leftEncoder.getCurrentPosition(); //updates the encoder's position inside the loop


        }

        while ((POWlocation > (1-tolerance) * newPOWTarget) && (POWlocation < (1+tolerance) * newPOWTarget)){
            telemetry.addData("are we there yet?", "yes");
            telemetry.addData("newPOWTarget:", newPOWTarget);
            telemetry.addData("POW location: ", POWlocation);
            telemetry.addData("leftEncoderPos:", robot.leftEncoder.getCurrentPosition());

            telemetry.update();

            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);

            POWlocation = robot.leftEncoder.getCurrentPosition(); //updates the encoder's position inside the loop
        }


    }


}


