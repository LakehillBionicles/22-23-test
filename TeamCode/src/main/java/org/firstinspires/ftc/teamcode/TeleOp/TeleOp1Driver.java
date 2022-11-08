package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TemaruHardware.armSpeed;
import static org.firstinspires.ftc.teamcode.TemaruHardware.closeHandPos;
import static org.firstinspires.ftc.teamcode.TemaruHardware.openHandPos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TemaruHardware;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentHashMap;

@TeleOp
//@Disabled
public class TeleOp1Driver extends LinearOpMode {
    TemaruHardware robot = new TemaruHardware();


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double startHeading;
    boolean isMoving;

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()) {


        if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 ||gamepad1.left_stick_y < -0.2){
            //if you move the left joystick, run the DRIVE method until the joystick goes back to "zero"
            while(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 ||gamepad1.left_stick_y < -0.2) {
                drive();
            }
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }else if(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15){
            //if you move the right joystick, run the TURN method until the joystick goes back to "zero"
            while(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15){
            turn();
            }
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }else{
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }
             //NEED
            //if you press a bumper, run the ARMLIFT method until the bumper is "unpressed"
            armLift();
            doLights();
            //hand();
            servoHand();
            setArmToHeight();

        }

       /* public void drive(double stickX double stickY) { //omni strafe ||Testing||
            double angle = Math.atan2(stickY, stickX);
            double magnitude = Math.sqrt(Math.pow(stickY, 2) + Math.pow(stickX, 2));
            if (stickX > 0.2 || stickX < -0.2 || stickY < -0.2 || stickY > 0.2) {
                isMoving = true;
                fsd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);//cos maybe?
               bpd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);
               fpd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
                bsd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
            }*/
    }

    public void turn() {
        if (gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15) {    //clockwise
            //isMoving = true; Don't know what this does might need it
            robot.fpd.setPower(-gamepad1.right_stick_x * speed);
            robot.bpd.setPower(gamepad1.right_stick_x * speed);
            robot.fsd.setPower(gamepad1.right_stick_x * speed);
            robot.bsd.setPower(-gamepad1.right_stick_x * speed);
    /*}else if((gamepad1.right_stick_x > -1 && gamepad1.right_stick_x < -0.2) && (gamepad1.right_stick_y < -.25 && gamepad1.right_stick_y > -1)){    //front port (left)
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(gamepad1.right_stick_y);
            robot.bsd.setPower(0.0);
            robot.fsd.setPower(gamepad1.right_stick_y);
        telemetry.addData("fp","");
        }else if(gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_x < 1 && gamepad1.right_stick_y > -1 && gamepad1.right_stick_y < -0.25){  //front star (right)
            robot.fpd.setPower(gamepad1.right_stick_y);
            robot.bpd.setPower(0.0);
            robot.bsd.setPower(gamepad1.right_stick_y);
            robot.fsd.setPower(0.0);
        telemetry.addData("fs","");
        }else if(gamepad1.right_stick_x > -1 && gamepad1.right_stick_x < -0.25  && gamepad1.right_stick_y > 0.25 && gamepad1.right_stick_y < 1){  //back port
            robot.fpd.setPower(gamepad1.right_stick_y);
            robot.bpd.setPower(0.0);
            robot.bsd.setPower(gamepad1.right_stick_y);
            robot.fsd.setPower(0.0);
        telemetry.addData("bp","");
        }else if(gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_x < 1 && gamepad1.right_stick_y < 1 && gamepad1.right_stick_y > 0.25){   //back star
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(gamepad1.right_stick_y);
            robot.bsd.setPower(0.0);
            robot.fsd.setPower(gamepad1.right_stick_y);
            telemetry.addData("bs","");
*/
        } else {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);

        }
    }

    //lk;jfl;asdjflk;asdhg;sdfj;lk

    public void drive() {
                if (gamepad1.left_stick_x > 0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {
                    robot.bpd.setPower(-gamepad1.left_stick_x);
                    robot.fpd.setPower(-gamepad1.left_stick_x);
                    robot.bsd.setPower(gamepad1.left_stick_x);
                    robot.fsd.setPower(gamepad1.left_stick_x);
                } else if (gamepad1.left_stick_x < -0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {
                    robot.bpd.setPower(-gamepad1.left_stick_x);
                    robot.fpd.setPower(-gamepad1.left_stick_x);
                    robot.bsd.setPower(gamepad1.left_stick_x);
                    robot.fsd.setPower(gamepad1.left_stick_x);
                } else if (gamepad1.left_stick_y > 0.2 && gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2) {//forward
                    robot.fpd.setPower(gamepad1.left_stick_y);
                    robot.bsd.setPower(-gamepad1.left_stick_y);
                    robot.fsd.setPower(gamepad1.left_stick_y);
                    robot.bpd.setPower(-gamepad1.left_stick_y);
                } else if (gamepad1.left_stick_y < -0.2 && gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2){//backward
                    robot.fpd.setPower(gamepad1.left_stick_y);
                    robot.bsd.setPower(-gamepad1.left_stick_y);
                    robot.fsd.setPower(gamepad1.left_stick_y);
                    robot.bpd.setPower(-gamepad1.left_stick_y);
                } else {
                    robot.fpd.setPower(0);
                    robot.bpd.setPower(0);
                    robot.fsd.setPower(0);
                    robot.bsd.setPower(0);
            }
        }



    public void armLift(){

        if (gamepad1.left_bumper) { //goes up
            robot.arm1.setPower(1);
            robot.arm2.setPower(1);
        } else if (gamepad1.right_bumper){
            robot.arm1.setPower(-0.4);
            robot.arm2.setPower(-0.4);
        } else if (!(gamepad1.left_bumper) && !(gamepad1.right_bumper)){
            robot.arm1.setPower(0.0);
            robot.arm2.setPower(0.0);
        } else {}
    }

    /*public void hand(){
        if (gamepad1.left_trigger > 0){
            robot.hand.setPower(1.0);
        } else if (gamepad1.right_trigger > 0){
            robot.hand.setPower(-1.0);
        } else {
            robot.hand.setPower(0.0);
        }
        //for reference w/in the setPos we had this instead: (robot.flippyBox.getPosition() + .081)

    }*/

    public void servoHand(){
        if (gamepad1.left_trigger > 0){
            robot.servoFinger.setPosition(0.0);
        } else {
            robot.servoFinger.setPosition(1.0);
        }
        //for reference w/in the setPos we had this instead: (robot.flippyBox.getPosition() + .081)

    }

    public void setArmToHeight(){
        while (gamepad1.a){
            if (robot.distSensorArm.getDistance(DistanceUnit.CM) > 50){
                robot.arm1.setPower(-0.1);
                robot.arm2.setPower(-0.1);

            } else if (robot.distSensorArm.getDistance(DistanceUnit.CM) < 50){
                robot.arm1.setPower(0.1);
                robot.arm2.setPower(0.1);

            } else {
              robot.arm1.setPower(0.0);
              robot.arm2.setPower(0.0);

            } //middle poll is 74
            //large poll is 100
            //hand distance is 15
        }
        while (gamepad1.a){
            if (robot.distSensorArm.getDistance(DistanceUnit.CM) > 50){
                robot.arm1.setPower(-0.1);
                robot.arm2.setPower(-0.1);

            } else if (robot.distSensorArm.getDistance(DistanceUnit.CM) < 50){
                robot.arm1.setPower(0.1);
                robot.arm2.setPower(0.1);

            } else {
                robot.arm1.setPower(0.0);
                robot.arm2.setPower(0.0);

            } //middle poll is 74
            //large poll is 100
            //hand distance is 15
        }



    }


    public void doLights(){
        if (robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        } else if (robot.distSensor.getDistance(DistanceUnit.CM) < 8){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        } else {
          robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }

    /*public void magnetMagic(){
        if (robot.magnet.isPressed()){ //is pressed at 1 cm
            telemetry.addData("it works", "yay");
            telemetry.update();
        } else {
            telemetry.addData("no work", "sad");
            telemetry.update();

        }


    }*/



        /* public void testDrive(){





    }*/


    }
