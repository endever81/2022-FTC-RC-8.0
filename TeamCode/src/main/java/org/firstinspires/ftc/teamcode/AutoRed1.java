package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name = "AutoRed1", group = "Automonous")

public class AutoRed1 extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      //"Ball",
      //"Cube",
      //"Duck",
      //"Marker"
      "Cap"
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

            tfod.setZoom(1.5, 25.0/9.0);
        }

     
       
       
        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
       
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize (parameters);
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        //sensorDistanceBack = hardwareMap.get(DistanceSensor.class, "back_distance");
       
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
       
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
       
        {
            sleep(50);
            idle();
        }
            
       
        telemetry.addData("Wheel Encoders",  "Starting at %7d :%7d :%7d :%7d",
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
        
        int w = 0;
        double leftValue = 0;
        boolean onLeft = true;
        boolean seeDuck = false;


//release pressure on the wheel
//servoWobble.setPosition(.7);

        if (opModeIsActive()) {
          //  while (opModeIsActive()) {
          if (tfod != null) {
          while(w < 2){
                //if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                      
                       if (recognition.getLabel() != "Duck")
                                 { telemetry.addData(">", "I Do not See a Duck"); 
                                    //duckPosition++;
                                 }
                        else
                        {
                            seeDuck = true;
                            leftValue = recognition.getLeft();
                            if (leftValue < 275)
                            {
                                //telemetry.addData(">", "Duck is on 1");
                                onLeft = true;
                            }
                            else
                            {
                                //telemetry.addData(">", "Duck is on 2");
                                onLeft = false;
                            }
                        }
                        i++;
                        w++;
                      }
                      w++;
                      telemetry.update();
                    }
                }
            }
                
                    
                    
                      
                      if (seeDuck == true && onLeft == true)
                      {
                          telemetry.addData(">","Duck is on 1");
                          telemetry.update();
                          //duck is on left and place block on bottom
                          
                          //robot.grabber.setPosition(0);
                        
                        sleep(750);
                        
                        lift(.2, 5);
                        
                        
                        gyroDrive (.3, 13, 0);
                        sleep(1000);
                        
                        gyroTurn (.3, 199.5);
                         sleep(1000);
                        gyroStrafe (.3, 24, 199.5);
                         sleep(1000);
                        robot.spinner.setPower(.3);
                        
                        sleep(4500);
                        
                        robot.spinner.setPower(0);
                        
                        
                        gyroStrafe(.3, -30, 180);
                        
                        gyroTurn(.3, -40);
                        
                        gyroDrive(.3, 18, -40);
                        
                        //robot.grabber.setPosition(.35);
                        
                        sleep(1000);
                        
                        gyroDrive(.5, -15, -40);
                        
                        gyroTurn(.3, -95);
                        
                       // gyroDrive (.4, 85, -95);
                      }
                        
                        
                        
                        
                        
                        
                      if (seeDuck == true && onLeft == false)
                      {
                          telemetry.addData(">","Duck is on 2");
                          telemetry.update();
                          //duck is on middle and place block on middle
                           //robot.grabber.setPosition(0);
                        
                        sleep(750);
                        
                       
                         lift(.2, 3);
                        
                        gyroDrive (.3, 13, 0);
                        sleep(1000);
                        
                        gyroTurn (.3, 199.5);
                         sleep(1000);
                        gyroStrafe (.3, 24, 199.5);
                         sleep(1000);
                        robot.spinner.setPower(.3);
                        
                        sleep(4500);
                        
                        robot.spinner.setPower(0);
                        
                        
                        gyroStrafe(.3, -30, 180);
                        
                        gyroTurn(.3, -40);
                         lift(.2, 6);
                        gyroDrive(.3, 18, -40);
                        
                        //robot.grabber.setPosition(.35);
                        
                        sleep(1000);
                        
                        gyroDrive(.5, -15, -40);
                        
                        gyroTurn(.3, -95);
                        
                       // gyroDrive (.4, 85, -95);
                          
                      }
                      
                      
                      
                      
                      
                      
                      if (seeDuck == false)
                      {
                          telemetry.addData(">","Duck is on 3");
                          telemetry.update();
                          //duck is on right and place block on top
                          
                     //robot.grabber.setPosition(0);
                        
                        sleep(750);
                        
                        lift(.2, 3);
                        
                        
                        gyroDrive (.3, 13, 0);
                        sleep(1000);
                        
                        gyroTurn (.3, 199.5);
                         sleep(1000);
                        gyroStrafe (.3, 24, 199.5);
                         sleep(1000);
                        robot.spinner.setPower(.3);
                        
                        sleep(4500);
                        
                        robot.spinner.setPower(0);
                        
                        
                        gyroStrafe(.3, -30, 180);
                        
                        gyroTurn(.3, -40);
                         lift(.2, 9);
                        gyroDrive(.3, 18, -40);
                        
                        //robot.grabber.setPosition(.35);
                        
                        sleep(1000);
                        
                        gyroDrive(.5, -15, -40);
                        
                        gyroTurn(.3, -95);
                        
                      //  gyroDrive (.4, 85, -95);

                    
                    
                    
                      }
                      
                      
                      
                      
                      
                      
        }

        if (tfod != null) {
            tfod.shutdown();
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
       tfod.loadModelFromFile("/sdcard/FIRST/tflitemodels/myCustomFreightFrenzyModel.tflite", LABELS);
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
        
        newLiftTarget = robot.arm.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));
        
        robot.arm.setTargetPosition(newLiftTarget);
        
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        runtime.reset();
        robot.arm.setPower(Math.abs(power));
        
         while (opModeIsActive() &&
                     robot.arm.isBusy()) {
        telemetry.addData("Lift", "Running at %7d",
                        robot.arm.getCurrentPosition());
                telemetry.update();
        
    }
    robot.arm.setPower(0);
    robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
}
    
}

 


   
}
