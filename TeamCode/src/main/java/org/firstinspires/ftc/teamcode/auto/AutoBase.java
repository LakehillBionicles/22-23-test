package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TemaruHardware;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AutoBase extends LinearOpMode {

    public TemaruHardware robot = new TemaruHardware();
    BNO055IMU imu;
    Orientation angles;


    static final double FEET_PER_METER = 3.28084;


    public int portEncoderPos = 0;
    public int starEncoderPos = 0;

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = (0.11111); // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = 1.2 * 4 * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));

    static final String sleeveColor = "";

    private double spedAdjust = .15;
    private int boundBE = 5;

    public final String VUFORIA_KEY = "AZjeccj/////AAABmZ7TkGdQaE90s4Gyo3b9T6oMtsulwtj5kAdhfhIabefDBj9bL1HNlKjYyp+p20rz5XXI3XDI+LJhqiNDUymG5F9OnRzuEMCWrAiD+KapcXmFnFqQE/1KtAdlOTLURn2zaOPk9yYQQnRuk4mKoIMNFSHbvD5jCcAEb2Xd6fCeFPXfUqof2JWKSklygJqup0mgtWOPlxb+PdPgRuGeSzTyZtOCuyGzny5vUTnno/ShUCH2Am56oJUwzvNJS22oBn1dwsPiNIZBJK/EkHfDzJPkxDLMQGP0r2FMDheJRy+nU/xQ///p26LxrG6Gm3MT1Wal7tVigS1IJEB0B+eoqK+6LBlRvDf+CFCBj9nXY7eIy9I1";


    /*** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.*/
    public VuforiaLocalizer vuforia;


    /*** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.*/
    public TFObjectDetector tfod;


    @Override
    public void runOpMode() {
//We might put the next lines in runOpMode in Startup
        startUp();
    }


    public void startUp() {
        robot.init(hardwareMap);

        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        setMotorDir();
        //setMotorDirStrafe();

        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initVuforia();

        //initTfod();

        telemetry.addData("Vision is Ready", ")");
        telemetry.update();


    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {  //forward back and turn with encoder, always do 3 less

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDir();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            telemetry.addData("counts per inch", this.COUNTS_PER_INCH);
            telemetry.update();

            newLeftTarget = (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = (int) ((rightInches) * this.COUNTS_PER_INCH);

            telemetry.addData("counts to run", newLeftTarget);
            telemetry.update();

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //give the motors a 2% error allowance
            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // reset the timeout time and start motion.
            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy() && robot.fsd.isBusy() && robot.bpd.isBusy() && robot.bsd.isBusy() &&
                    opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2)) ||
                            (Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) {
                //empty loop body that keeps running leaving motors running until time is up
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(40);
        }
    }
    //DriveUntilDist doesn't work no idea why. Motors just don't turn on
    public void driveUntilDist(double speed, double timeoutS) {  //method to drive until dist. sensor reads.../


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            runtime.reset();
            robot.fpd.setPower(-0.2);
            robot.fsd.setPower(-0.2);
            robot.bsd.setPower(0.2);
            robot.bpd.setPower(0.2);
            //changed all below to pos (both f and s were neg)


            while (robot.distSensorHorizontal.getDistance(DistanceUnit.CM) > 9 && runtime.seconds()<timeoutS && opModeIsActive()) {//Is 13 correct?
                telemetry.addData("horizontal", robot.distSensorHorizontal.getDistance(DistanceUnit.CM));
                telemetry.update();
                //Do nothing letting motors run until distance sensor sees a cone
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Turn off RUN_TO_POSITION
            /*robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
            sleep(40);
        }
    }

    public void setMotorDirTurn(){
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //Positive is counterclockwise
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void turn(double speed, double portInches, double starInches, double timeoutS){//Positive is counterclockwise. 1 inch = 6 degrees
            int newPortTarget = 0;
            int newStarTarget = 0;
            setMotorDirTurn();

        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newPortTarget = robot.fpd.getCurrentPosition() + (int) ((portInches * this.COUNTS_PER_INCH)/6.67);
            newStarTarget = robot.bsd.getCurrentPosition() + (int) ((starInches * this.COUNTS_PER_INCH)/6.67);

            robot.fpd.setTargetPosition(newPortTarget);
            robot.fsd.setTargetPosition(newStarTarget);
            robot.bpd.setTargetPosition(newPortTarget);
            robot.bsd.setTargetPosition(newStarTarget);


            int turnNT1 = (int) (newPortTarget * 0.981); //fs
            int turnNT2 = (int) (newStarTarget * 0.981); //bs       //I don't know what NT Does


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));  //-
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));  //-
            robot.bpd.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    // ( front_star_wheel.isBusy() && front_port_wheel.isBusy()))
                    ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(turnNT1)) || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(turnNT2)))) { // took this out for now || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newPortTarget, newStarTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.fpd.getCurrentPosition(),
                        robot.bsd.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move
        }



    }





    public void sideways(double speed, double frontInches, double backInches, double timeoutS) { //positive is right
        int newFrontTarget = 0;
        int newBackTarget = 0;

        setMotorDirStrafe();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newFrontTarget = robot.fsd.getCurrentPosition() + (int) (frontInches * this.COUNTS_PER_INCH);
            newBackTarget = robot.bsd.getCurrentPosition() + (int) (backInches * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newFrontTarget);
            robot.fsd.setTargetPosition(newFrontTarget);
            robot.bpd.setTargetPosition(newBackTarget);
            robot.bsd.setTargetPosition(newBackTarget);


            int NT1 = (int) (newFrontTarget * 0.981); //fs
            int NT2 = (int) (newBackTarget * 0.981); //bs


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));  //-
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));  //-
            robot.bpd.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    // ( front_star_wheel.isBusy() && front_port_wheel.isBusy()))
                    ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT1)) || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) { // took this out for now || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontTarget, newBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.fsd.getCurrentPosition(),
                        robot.bsd.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move
        }


    }

    public void correctSideways(double speed) {

        double sped = (speed - spedAdjust);

        if (robot.bsd.getCurrentPosition() > boundBE) { // too much forward side

            while (robot.bsd.getCurrentPosition() > boundBE) {
                //decrease front side motor speed for adjustments, at spEd
                robot.fpd.setPower(sped);
                robot.fsd.setPower(sped);

            }

            //put front side motor speed back where it should be, at spEEd
            robot.fsd.setPower(speed);
            robot.fpd.setPower(speed);

        } else if (robot.bsd.getCurrentPosition() < -boundBE) {   //too much back side

            while (robot.bsd.getCurrentPosition() < -boundBE) {
                //decrease back side motor speed for adjustments, at spEd
                robot.bsd.setPower(sped);
                robot.bpd.setPower(sped);

            }

            //put back side motor speed back where it should be, at spEEd
            robot.bsd.setPower(sped);
            robot.bpd.setPower(sped);

        }
    }

    public void driveUntilTouch(double speed) { //function that drives until both touch sensors are pushed
        if (opModeIsActive()) {

            runtime.reset();

            setMotorDir();

            robot.fpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (!(robot.touchSensorStar.isPressed()) || !(robot.touchSensorPort.isPressed())) {
                //Do nothing letting motors run until distance sensor sees a cone
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            /*robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(40);*/
        }
    }



        public void armLift ( double speed, double inches, double timeoutS){ //right now this function only lifts to the highest pole



        inches = inches / (2*3.14*23.8) * 560;
            // inches = inches*((560*23.8)/25.4);
            robot.frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int newTarget = 0;

            if (opModeIsActive()) {

                newTarget = (int) inches; //inches * this.COUNTS_PER_INCH?
                robot.frontEncoder.setTargetPosition(newTarget);
                robot.arm2.setTargetPosition(newTarget);
                robot.frontEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int NT = (int) (newTarget * 0.981); //fp
                runtime.reset();
                robot.frontEncoder.setPower(Math.abs(speed));
                robot.arm2.setPower(Math.abs(speed*0.9));//0.9 is encoder ticks of arm 1 over arm2
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && robot.frontEncoder.isBusy() &&
                        ((Math.abs(robot.frontEncoder.getCurrentPosition()) < Math.abs(NT)))) {
                }
                robot.frontEncoder.setPower(0.6);
                robot.arm2.setPower(0.6);
                robot.frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }

        public void armStayUp(){
        while (opModeIsActive()){
            robot.frontEncoder.setPower(0.1);
            robot.arm2.setPower(0.1);
            }
    }

        public void handDrop () {
            robot.servoFinger.setPosition(0.0);
            sleep(1500); //this wait may need to be longer
            robot.servoFinger.setPosition(1.0);
        }



        public void setMotorDir () { //make sure correct - not 100% sure
            robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //forward
            robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD); //neg??

        }

        public void setMotorDirStrafe () {
            robot.fsd.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.bpd.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    public void senseColorTelemetry(){ //drive until see not green
        while (opModeIsActive()){
        telemetry.addData("redStar", robot.colorSensorStar.red());
        telemetry.addData("greenStar", robot.colorSensorStar.green());
        telemetry.addData("blueStar", robot.colorSensorStar.blue());

        telemetry.addData("redPort", robot.colorSensorPort.red());
        telemetry.addData("greenPort", robot.colorSensorPort.green());
        telemetry.addData("bluePort", robot.colorSensorPort.blue());

        telemetry.update();

    }
    }

    public String senseColorsStar () {
                String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
            if (robot.colorSensorStar.red() > (robot.colorSensorStar.blue()) && robot.colorSensorStar.red() > (robot.colorSensorStar.green())) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorStar.blue() > (robot.colorSensorStar.red()) && robot.colorSensorStar.blue() > (robot.colorSensorStar.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorStar.green() > (robot.colorSensorStar.red()) && robot.colorSensorStar.green() > (robot.colorSensorStar.blue())) {
                colorStar = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorStar = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorStar = "no go";

            }

        }
        return colorStar;
    }

    public String senseColorsPort(){
        String colorPort = "blank";

        while (opModeIsActive()&& colorPort.equals("blank")){

            if (robot.colorSensorPort.red() > (robot.colorSensorPort.blue()) && robot.colorSensorPort.red() > (robot.colorSensorPort.green())){
                colorPort = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorPort ="red";


            } else if (robot.colorSensorPort.blue() > (robot.colorSensorPort.red()) && robot.colorSensorPort.blue() > (robot.colorSensorPort.green())){
                colorPort = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorPort ="blue";

            } else if (robot.colorSensorPort.green() > (robot.colorSensorPort.red()) && robot.colorSensorPort.green() > (robot.colorSensorPort.blue())){
                colorPort = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorPort = "green";

            }else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorPort = "no go";
            }
        }

            return colorPort;
    }






    public void blueParkingMethod (){



    }

    /*public void initVuforia() {

         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }*/

    /*public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(1, 16.0 / 9.0);
        tfod.activate();
    }*/


    }

