package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.TemaruHardware;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.List;

public class AutoBase extends LinearOpMode {

    TemaruHardware robot = new TemaruHardware();
    BNO055IMU imu;
    Orientation angles;

    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = (0.11111); // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = 1.2*4*((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));

    public final String VUFORIA_KEY = "AZjeccj/////AAABmZ7TkGdQaE90s4Gyo3b9T6oMtsulwtj5kAdhfhIabefDBj9bL1HNlKjYyp+p20rz5XXI3XDI+LJhqiNDUymG5F9OnRzuEMCWrAiD+KapcXmFnFqQE/1KtAdlOTLURn2zaOPk9yYQQnRuk4mKoIMNFSHbvD5jCcAEb2Xd6fCeFPXfUqof2JWKSklygJqup0mgtWOPlxb+PdPgRuGeSzTyZtOCuyGzny5vUTnno/ShUCH2Am56oJUwzvNJS22oBn1dwsPiNIZBJK/EkHfDzJPkxDLMQGP0r2FMDheJRy+nU/xQ///p26LxrG6Gm3MT1Wal7tVigS1IJEB0B+eoqK+6LBlRvDf+CFCBj9nXY7eIy9I1";


    /*** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.*/
    public VuforiaLocalizer vuforia;


    /*** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.*/
    public TFObjectDetector tfod;


    public AutoBase() {
    }

    @Override
    public void runOpMode(){
        startUp();
    }

    public void startUp(){
        robot.init(hardwareMap);

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
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))))
            {
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
            sleep(40);
        }
    }

    public void sideways(double speed, double frontInches, double backInches, double timeoutS) { //positive is left
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

    public void setMotorDir() { //make sure correct - not 100% sure
        robot.fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setMotorDirStrafe(){ //make sure correct
        robot.fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
    }

   /* public void turn90Left(double speed, double timeoutS){ //double degrees for later to input in turn
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMotorDir();

        if (opModeIsActive()){

            new_fpdTarget = (int)//encoder counts go here
            new_fsdTarget = (int)
            new_bpdTarget = (int)
            new_bsdTarget = (int)

            robot.fpd.setTargetPosition(new_fpdTarget);
            robot.fsd.setTargetPosition(new_fsdTarget);
            robot.bpd.setTargetPosition(new_bpdTarget);
            robot.bsd.setTargetPosition(new_bsdTarget);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int fpdPost = (int) (new_fpdTarget * 0.981);
            int fsdPost = (int) (new_fsdTarget * 0.981);
            int bpdPost = (int) (new_bpdTarget * 0.981);
            int bsdPost = (int) (new_bsdTarget * 0.981);

            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy() && robot.fsd.isBusy() && robot.bpd.isBusy() && robot.bsd.isBusy() &&
                    opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(fpdPost)) ||
                            (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(fsdPost)) ||
                            (Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(bpdPost)) ||
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(bsdPost))))
            {
                //empty loop body that keeps running leaving motors running until time is up
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bpd.setPower(0);
            robot.bsd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }







    } */

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(1, 16.0 / 9.0);
        tfod.activate();
    }
    public void updateAngles() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); //NullPointerException!!??
    }







}
