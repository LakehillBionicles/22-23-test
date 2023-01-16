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


public class TeleOp2Drivers extends LinearOpMode { //gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances
    TemaruHardware robot = new TemaruHardware();


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double speedStrafe = 0.7;
    double startHeading;
    boolean isMoving;

    ////////////////////////////////////////////////////  ELBOW VARIABLES ///////////////////////////////////////////////
    public double elbowDenominator;
    public double elbowPosition;
    public double elbowError;
    public double lastElbowError;
    public double elbowTime = 0;
    public double newElbowTarget;
    public double elbowDeriv;
    public double elbowIntegralSum = 0;
    public double elbowIntegralSumLimit = 0.25;

    public double elbowKp = 1.0;
    public double elbowKd = 1.0;
    public double elbowKi = 1.0;

    public double elbowPower;

    public ElapsedTime runtime = new ElapsedTime(); //might cause an error (not sure)


    public void runOpMode() {

        robot.init(hardwareMap);

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            }else if(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15&& gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0){
                //if you move the right joystick, run the TURN method until the joystick goes back to "zero"
                while(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15&& gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0){
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

            if(gamepad2.dpad_up){


                fourBarPID2(.5, 25, 5);


            } else {
                robot.arm2.setPower(0.0);
            }
            //NEED
            //if you press a bumper, run the ARMLIFT method until the bumper is "unpressed"
            armLift();
            servoHand();
            servoHandZeroInput();
            setArmToHeight();
            displayDistance();
            //fourBar();
            //fourBarPID2();


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
        if (gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15 && gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0) {    //clockwise
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

    public void drive() {


        if (gamepad1.left_stick_x > 0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {//right

            robot.bpd.setPower(- (speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8))));
            robot.fpd.setPower(- (speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.bsd.setPower(speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8)));
            robot.fsd.setPower(speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8)));
        } else if (gamepad1.left_stick_x < -0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {//left

            robot.bpd.setPower(- (speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8))));
            robot.fpd.setPower(- (speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.bsd.setPower(speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8)));
            robot.fsd.setPower(speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8)));
        } else if (gamepad1.left_stick_y > 0.2) {//forward
            robot.fpd.setPower(gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));// && (gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2)
            robot.bsd.setPower(-gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));
            robot.fsd.setPower(gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
            robot.bpd.setPower(-gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
        } else if (gamepad1.left_stick_y < -0.2){//backward
            robot.fpd.setPower(gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));
            robot.bsd.setPower(-gamepad1.left_stick_y- (gamepad1.right_stick_x * .8));//(gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2)
            robot.fsd.setPower(gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
            robot.bpd.setPower(-gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
        } else {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }
    }


    public void armLift(){
        if (gamepad2.left_stick_y < -0.2) {   //used bumpers here previously//
            robot.BOW.setPower(1);
            robot.SOW.setPower(-1);
        } else if (gamepad2.right_stick_y > 0.2){
            robot.BOW.setPower(-1);
            robot.SOW.setPower(1);
        } else if (!(gamepad2.left_stick_y < -0.2) && !(gamepad2.right_stick_y > 0.2) && !gamepad2.a && !gamepad2.b && !gamepad2.y && !gamepad2.x){
            robot.BOW.setPower(0.0);
            robot.SOW.setPower(0.0);
        } else {}
    }

    public void setArmToHeight(){
        if (gamepad2.a) { //cone distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 25) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 25) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.x){ //small distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)  - 60) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 60) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.y){ //medium distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.b){ //large distance -- might need to comment out
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85))));
        }


        /* MATH EXPLAINED BELOW /////////////////////////////////////////////////////////////////////////////////////////

         * 2 * --> adds more power to the arm
         * - sign --> fixes direction
         * sin (((current distances of all sensors added together - target distance) * pi/4) /total height of arm)
         *
         * *////////////////////////////////////////////////////////////////////////////////////////////////////////


        //(sin(|current - target|) / max height ) * pi / 4

        //target pos - current pos = how far away
        //set speed based on
        //power is function of __ or power is minus function of ___
        //absolute value around difference

        //if current - target is here this power
        // if current - target is here this power


    }








    public void fourBarPID2(double maxElbowPower, double inputTargetPos, double elbowTol) { //uses 2-3 predetermined targets with no way for driver to adjust
        elbowPosition = robot.arm2.getCurrentPosition();

        newElbowTarget = (inputTargetPos);

        elbowError = (newElbowTarget - elbowPosition);

        telemetry.addData("elbow pos", elbowPosition);
        telemetry.addData("new elbow target", newElbowTarget);
        telemetry.addData("error", elbowError);
        telemetry.addData("elbow time", getRuntime());
        telemetry.update();

        while (Math.abs(elbowError) > (elbowTol)){

            elbowTime = getRuntime();

            elbowDenominator = Math.max(Math.abs(elbowPower), 1);


            telemetry.addData("in while loop", "yay");
            telemetry.addData("elbowPower", elbowPower);
            telemetry.addData("elbowDenominator", elbowDenominator);
            telemetry.addData("power function", (elbowPower * maxElbowPower / elbowDenominator));
            telemetry.addData("runtime", elbowTime);
            telemetry.addData("elbow pos", elbowPosition);
            telemetry.addData("new elbow target", newElbowTarget);
            telemetry.addData("error", elbowError);
            telemetry.update();

            elbowPower = ((elbowKd * elbowDeriv) + (elbowKi * elbowIntegralSum) + (elbowKp * Math.signum(elbowError)));

            robot.arm2.setPower(elbowPower * maxElbowPower / elbowDenominator);


            if (Math.abs(elbowError) > Math.abs(elbowTol)){
                elbowDeriv = ((elbowError - lastElbowError) / elbowTime);
                elbowIntegralSum = (elbowIntegralSum + (elbowError * elbowTime));
                if (elbowIntegralSum > elbowIntegralSumLimit){
                    elbowIntegralSum = elbowIntegralSumLimit;
                }
                if (elbowIntegralSum < -elbowIntegralSumLimit){
                    elbowIntegralSum = -elbowIntegralSumLimit;
                }
                elbowPower = ((elbowKd * elbowDeriv) + (elbowKi * elbowIntegralSum) + (elbowKp * Math.signum(elbowError)));
                robot.arm2.setPower(elbowPower * maxElbowPower / elbowDenominator);
            }
            else{
                elbowPower = 0;
                robot.arm2.setPower(elbowPower);
            }

            elbowPosition = robot.arm2.getCurrentPosition();
            elbowError = (newElbowTarget - elbowPosition);


        }

        telemetry.addData("out of while loop", "yay");
        telemetry.update();

        robot.arm2.setPower(0);

    }





       /* robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double FourBarCurrentPos;
        double FourBarTarget;

        FourBarCurrentPos = robot.arm2.getCurrentPosition();
        FourBarTarget = 0.0;


        if (gamepad2.dpad_up) {
            FourBarTarget = 13; //replace this value with an actual number
            robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int NT = (int) (FourBarTarget * 0.981); //fp
            robot.arm2.setPower(1.0);
            robot.arm2.setPower(Math.sin((-113) - FourBarCurrentPos)* (3.1415 / 4));
            telemetry.addData("yay", "");
            telemetry.update();

        } else if (gamepad2.left_bumper) {
            FourBarTarget = 13; //replace this value with an actual number
            robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int NT = (int) (FourBarTarget * 0.981); //fp
            robot.arm2.setPower(1.0);

            robot.arm2.setPower(Math.sin((-113) - FourBarCurrentPos)* (3.1415 / 4));

        } else if (gamepad2.right_bumper) {
            FourBarTarget = 13; //replace this value with an actual number
            robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int NT = (int) (FourBarTarget * 0.981); //fp
            robot.arm2.setPower(1.0);

            robot.arm2.setPower(Math.sin((25) - FourBarCurrentPos)* (3.1415 / 4));

        } else {
            robot.arm2.setPower(0);

            //-113 at cone height to approx. 25 at cone drop height


        }*/


    //power to drive both up and down
    //track position using the encoder
    //power will be a function of error
    //error:
    //same sin function???????? please work???

    //need variable for targets and error in order to run to position








    public void fourBarEncoders(double speed, double leftInches, double rightInches,double timeoutS){
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //we switched arm2 and SOW, not sure why they're used together here
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (int) ((leftInches)* (288/360));
            newRightTarget = (int) ((rightInches)* (288/360));//288 is counts per revolution of core hex

            robot.SOW.setTargetPosition(newLeftTarget);
            robot.arm2.setTargetPosition(newRightTarget);

            robot.SOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // reset the timeout time and start motion.
            runtime.reset();

            robot.SOW.setPower(Math.abs(speed));
            robot.arm2.setPower(Math.abs(speed));

            while (robot.SOW.isBusy() && robot.SOW.isBusy() && opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ((Math.abs(robot.SOW.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.SOW.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.arm2.getCurrentPosition()) < Math.abs(NT2))))
            {
                //empty loop body
            }


            robot.SOW.setPower(0);
            robot.arm2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(40);
        }
    }

    public void servoHand(){
        if (gamepad2.right_trigger > 0){
            telemetry.addData("hand pos", "open");
            telemetry.update();
            robot.servoFinger.setPosition(0.0);

        } else if (gamepad2.left_trigger > 0){
            telemetry.addData("hand pos:", "closed");
            telemetry.update();
            robot.servoFinger.setPosition(0.45);
        } else {
        }
    }

    public void servoHandZeroInput(){
        if ((robot.distSensorHand.getDistance(DistanceUnit.CM) < 13) && (robot.colorSensorHand.equals("red") || robot.colorSensorHand.equals("blue"))){
            telemetry.addData("i see:", "cone");
            telemetry.update();
            robot.servoFinger.setPosition(0.45);
        }


    }




    public void displayDistance(){
        telemetry.addData("upper arm:", robot.distSensorArm.getDistance(DistanceUnit.CM));
        telemetry.addData("lower arm:", robot.distSensorLowerArm.getDistance(DistanceUnit.CM));
        telemetry.addData("middle arm:", robot.distSensorHighArm.getDistance(DistanceUnit.CM));
        telemetry.addData("distance:", robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM));

        telemetry.addData("arm pos", robot.arm2.getCurrentPosition());


        telemetry.addData("hand", robot.distSensorHand.getDistance(DistanceUnit.CM));
        telemetry.update();



        //small distance 39
        //medium distance 47
        //big distance
        // pick up cone from ground dist 15



    }




    public void doLights(){
        if (robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            //} else if (robot.distSensorHand.getDistance(DistanceUnit.CM) < 8){
            // robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        } else if (robot.touchSensorArm.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED); //this should make it works with the dist. sensor
        }else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }




}
