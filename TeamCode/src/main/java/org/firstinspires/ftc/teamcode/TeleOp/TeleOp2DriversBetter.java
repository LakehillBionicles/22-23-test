package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.TemaruHardware.armSpeed;
import static org.firstinspires.ftc.teamcode.TemaruHardware.closeHandPos;
import static org.firstinspires.ftc.teamcode.TemaruHardware.openHandPos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TemaruHardware;

@TeleOp
//@Disabled


//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////


public class TeleOp2DriversBetter extends LinearOpMode {
    TemaruHardware robot = new TemaruHardware();

    //the below variables are private to keep everything within this class

    //gamepad 1 (drive) variable
    private double drivePower;
    private double strafePower;
    private double rotatePower;

    //gamepad 2 (arm/elbow/hand) variable
    private double armPower;
    private double elbowPower;
    private double handPos;

    private double fpdPower;
    private double bpdPower;
    private double fsdPower;
    private double bsdPower;

    private double teleDenom;


    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("update date", "____");
        telemetry.update();


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            /*
            first test:
                gamepad left y =
                gamepad left x =
                gamepad right x =



            */
            if (gamepad1.left_trigger>0) {
                drivePower = gamepad1.left_stick_y / 2;
                strafePower = -gamepad1.left_stick_x / 2;
                rotatePower = gamepad1.right_stick_x / 2;
            }else{
                drivePower = gamepad1.left_stick_y ;
                strafePower = -gamepad1.left_stick_x ;
                rotatePower = -gamepad1.right_stick_x ;
            }
            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////
            if (gamepad2.dpad_up) {
                elbowPower = 1;
            } else if (gamepad2.dpad_down) {
                elbowPower = -1;
            } else {
                elbowPower = 0;
            }

            armPower = -gamepad2.left_stick_y;

            if (gamepad2.left_trigger > 0) {
                handPos = 1; //this is the dropPos and our number is wrong :|
            } else {
                handPos = 0; //again not the right number and this should be the grabPos
            }

            ////////////////////////////////////////////////////////// MATH //////////////////////////////////////////////////////////
            fpdPower = 1.25 * (drivePower + strafePower + rotatePower/2); //check to see if multiplying by 1.25 fixes the lag for fpd
            bpdPower = drivePower - strafePower + rotatePower/2;
            fsdPower = drivePower - strafePower - rotatePower/2;
            bsdPower = drivePower + strafePower - rotatePower/2;

            teleDenom = Math.max(Math.max(Math.abs(fpdPower), Math.abs(bpdPower)), Math.max(Math.abs(fsdPower), Math.abs(bsdPower)));

            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////
            robot.fpd.setPower(fpdPower/teleDenom);
            robot.fsd.setPower(fsdPower/teleDenom);
            robot.bpd.setPower(-(bpdPower/teleDenom));
            robot.bsd.setPower(-(bsdPower/teleDenom));


            robot.BOW.setPower(armPower);
            robot.SOW.setPower(-armPower);  //are these the right motors?

            robot.arm2.setPower(elbowPower);

            robot.servoFinger.setPosition(handPos);



        }
    }




}
