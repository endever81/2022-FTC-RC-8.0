package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Red Left", group = "Automonous")

public class RedLeft extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AcSWQyT/////AAABmXXMracDmUH2jST1AK/jjlB9bLitWfl+EeHaTDyQwcEVZ0/pIKzSLLnKb++x6kKcTYJnrBSWXcbq43Pa/x7v0cEfSLljqPHAntPUwrcTa7Ag5MR/KnSvxThO52HlzZ1T9S5JJtViLz5JLvrm8siLeJIK9uPiqKkYG3IkLBtXnHMLjB/4kfn5zfjnDzpwjgl+2bNzztz/dM91B1u6kroe/QCHWSWBeEgG8vJnVG/ko1aVkiroqaR/al9iui+lPRzAMMcSMKgxxW5sV5DcVdKWVXJq309wm2lUDXKT/4V3C8w48/KkI1J/B7YdB5um6TPCo6Jt8eaczYV3cuX3HmStOvTH1S5ixph1K/9TmyPoKhas";

 
    private VuforiaLocalizer vuforia;

   
    private TFObjectDetector tfod;
   
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverest 40:1
    static final double DRIVE_GEAR_REDUCTION = .5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double STRAFE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;
   
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.02;     // Larger is more responsive, but also less stable
    //int size = 0;
   
   

// Calls up IMU (Inertial Measruement Unit within REV Hub)
    BNO055IMU imu;
   
    Orientation angles;

    HardwareRobot robot = new HardwareRobot();

   

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.5, 10.0 / 9.0);
        }


        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        //sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "back_distance");

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        telemetry.addData("Wheel Encoders", "Starting at %7d :%7d :%7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition()
        );


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Distance (cm)",String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));


        telemetry.update();


        waitForStart();

        //Identify which signal is being displayed.

        int x = 0;
        if (opModeIsActive()) {
            //  while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    x = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());


                        if (recognition.getLabel() == "1 Bolt") {
                            x = 1;
                        }
                        if (recognition.getLabel() == "2 Bulb") {
                            x = 2;
                        }
                        if (recognition.getLabel() == "3 Panel") {
                            x = 3;
                        }

                    }


                }
                telemetry.update();
            }

        }


        if (x == 1) {
            telemetry.addData(">", "Left");
            telemetry.update();
            //Position 1 - LEFT

            robot.grabber.setPosition(.2);

            sleep(750);

            lift(.2, 2);
            gyroStrafe(.5, -20, 0);
            gyroDrive(.3, 25, 0);

            sleep(1000);

            //gyroTurn(.3, 199.5);


        }


        if (x == 2) {
            telemetry.addData(">", "Center");
            telemetry.update();
            //Position 2 - MIDDLE

            robot.grabber.setPosition(.2);

            sleep(750);

            lift(.2, 2);
            gyroDrive(.3, 25, 0);

            sleep(1000);

            //gyroTurn(.3, 199.5);



        }


        if (x == 3) {
            telemetry.addData(">", "Right");
            telemetry.update();
            //Position 3 - RIGHT

            robot.grabber.setPosition(.2);

            sleep(750);

            lift(.2, 2);
            gyroStrafe(.5, 20, 0);
            gyroDrive(.3, 25, 0);

            sleep(1000);

            //gyroTurn(.3, 199.5);

        }
        if (x == 0) {
            telemetry.addData(">", "Did not See");
            telemetry.update();

            robot.grabber.setPosition(.2);

            sleep(750);

            lift(.2, 2);
            gyroStrafe(.5, -25, 0);

        }

    }




public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
   
        public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
       
            robot.rightFrontDrive.setPower(rightSpeed);
            robot.leftFrontDrive.setPower(leftSpeed);
            robot.rightRearDrive.setPower(rightSpeed);
            robot.leftRearDrive.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
   
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
   
   
public void gyroDrive ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
           

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
               rightSpeed = speed + steer;
               


                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
 
 
             
 
 
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);
               

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
       
       
     }  
     
     public void gyroDriveNoBrake ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
           

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
               rightSpeed = speed + steer;
               


                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
 
 
             
 
 
                robot.rightFrontDrive.setPower(rightSpeed);
            robot.leftFrontDrive.setPower(leftSpeed);
            robot.rightRearDrive.setPower(rightSpeed);
            robot.leftRearDrive.setPower(leftSpeed);
               

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

     


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
       
       
     } 
     
     public void gyroStrafe ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() - moveCounts;
            

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
           

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy()))  {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
               rightSpeed = speed + steer;
               


                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
 
                robot.rightFrontDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
                robot.leftRearDrive.setPower(rightSpeed);
                
               

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        }
       
    }
  



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
       int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.68f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }






boolean onHeadingShoot(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        // Send desired speeds to motors.

            robot.rightFrontDrive.setPower(rightSpeed);
            robot.leftFrontDrive.setPower(leftSpeed);
            robot.rightRearDrive.setPower(rightSpeed);
            robot.leftRearDrive.setPower(leftSpeed);



//motorfeeder.setPower(1);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

   

public void lift(double power, double inches)
{
    int newLiftTarget;
    
    if (opModeIsActive()) {

        newLiftTarget = robot.lift.getCurrentPosition() + (int) (inches * (1140 / (3.5 * 3.1415)));

        robot.lift.setTargetPosition(newLiftTarget);

        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.lift.setPower(Math.abs(power));

        while (opModeIsActive() && robot.lift.isBusy()) {
            telemetry.addData("Lift", "Running at %7d",
                    robot.lift.getCurrentPosition());
            telemetry.update();

        }
        robot.lift.setPower(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
   
}
    
}

 


   

