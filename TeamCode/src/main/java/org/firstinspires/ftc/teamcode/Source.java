package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled

public class Source {


/*

public void motorsOff ()
    {
        motorFrontRight.setPower(0.0);
        motorFrontLeft.setPower(0.0);
        motorRearRight.setPower(0.0);
        motorRearLeft.setPower(0.0);
    }
    
    public void latchDown ()
    {
      servoLatchLeft.setPosition(0.50);
      servoLatchRight.setPosition(0.50);
    }
    
    public void latchUp ()
    {
      servoLatchLeft.setPosition(0.0);
      servoLatchRight.setPosition(1.0);

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
        
        motorFrontRight.setPower(rightSpeed);
            motorFrontLeft.setPower(leftSpeed);
            motorRearRight.setPower(rightSpeed);
            motorRearLeft.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + moveCounts;
            newRearRightTarget = motorRearRight.getCurrentPosition() + moveCounts;
            newRearLeftTarget = motorRearLeft.getCurrentPosition() + moveCounts;

            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorRearRight.setPower(speed);
            motorRearLeft.setPower(speed);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {

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
 
 
             
 
 
                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(leftSpeed);
                motorRearRight.setPower(rightSpeed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


public void lift(double power, double inches)
{
    int newLiftTarget;
    
    if (opModeIsActive()) {
        
        newLiftTarget = motorLift.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));
        
        motorLift.setTargetPosition(newLiftTarget);
        
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        runtime.reset();
        motorLift.setPower(Math.abs(power));
        
         while (opModeIsActive() &&
                     motorLift.isBusy()) {
        telemetry.addData("Lift", "Running at %7d",
                        motorLift.getCurrentPosition());
                telemetry.update();
        
    }
    motorLift.setPower(0);
    motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
}
    
}



public void encoderDrive(double speed,
                             double leftInches1, double leftInches2, double rightInches1,
                             double rightInches2, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
            newRearRightTarget = motorRearRight.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);
            newRearLeftTarget = motorRearLeft.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);

            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorRearRight.setPower(Math.abs(speed));
            motorRearLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    
   //GYRO STRAFE METHOD
public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + moveCounts;
            newRearRightTarget = motorRearRight.getCurrentPosition() + moveCounts;
            newRearLeftTarget = motorRearLeft.getCurrentPosition() - moveCounts;

            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorRearRight.setPower(speed);
            motorRearLeft.setPower(speed);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {


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
 
 
             
 
 
                motorFrontRight.setPower(leftSpeed);
                motorFrontLeft.setPower(speed);
                motorRearRight.setPower(speed);
                motorRearLeft.setPower(rightSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
    }

public void gyroStrafeColorRed ( double speed,  double angle, double alphaValue, double timeOutS) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
    
       motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
     //set power level for strafe
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(-speed);
        motorRearRight.setPower(-speed);
        motorRearLeft.setPower(speed);

    // Ensure that the opmode is still active
     if (opModeIsActive()) 
     {
       runtime.reset();
            //insert alpha value
        while (opModeIsActive() && sensorColor.alpha() > alphaValue && (runtime.seconds() < timeOutS)
)
          {        if (!opModeIsActive())
                            return;
                            
                            
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        
           Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
           telemetry.addData("Alpha", sensorColor.alpha());
           telemetry.addData("Encoder", motorFrontRight.getCurrentPosition());
          
          


           // adjust relative speed based on heading error.
           error = getError(angle);
           steer = getSteer(error, P_DRIVE_COEFF);

           // if driving in reverse, the motor correction also needs to be reversed
           //if (distance < 0)
           //         steer *= -1.0;

           leftSpeed = speed - steer;
           rightSpeed = speed + steer;
               


           //Normalize speeds if either one exceeds +/- 1.0;
           max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
           if (max > 1.0)
                {
                leftSpeed /= max;
                rightSpeed /= max;
                }
 

                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(-speed);
                motorRearRight.setPower(-speed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);



        }
        
    }
    
    
    
public void gyroDriveDistanceAverage ( double speed,  double angle, double distance, double timeOutS) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
    
       motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
     //set power level for strafe
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorRearRight.setPower(speed);
        motorRearLeft.setPower(speed);

    // Ensure that the opmode is still active
     if (opModeIsActive()) 
     {
       runtime.reset();
            //insert alpha value
        while (opModeIsActive() && ((sensorDistance1.getDistance(DistanceUnit.CM)+ sensorDistance2.getDistance(DistanceUnit.CM)+ sensorDistance3.getDistance(DistanceUnit.CM))/3) > distance && (runtime.seconds() < timeOutS))   
          {        if (!opModeIsActive()){
                            return;}
                            
                            
           telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));
            
           // adjust relative speed based on heading error.
           error = getError(angle);
           steer = getSteer(error, P_DRIVE_COEFF);

           // if driving in reverse, the motor correction also needs to be reversed
           //if (distance < 0)
           //         steer *= -1.0;

           leftSpeed = speed - steer;
           rightSpeed = speed + steer;
               


           //Normalize speeds if either one exceeds +/- 1.0;
           max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
           if (max > 1.0)
                {
                leftSpeed /= max;
                rightSpeed /= max;
                }
 

                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(leftSpeed);
                motorRearRight.setPower(rightSpeed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);



        }
        
    }



 public void gyroDriveDistance ( double speed,  double angle, double distance, double timeOutS) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
    
       motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
     //set power level for strafe
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorRearRight.setPower(speed);
        motorRearLeft.setPower(speed);

    // Ensure that the opmode is still active
     if (opModeIsActive()) 
     {
       runtime.reset();
            //insert alpha value
        while (opModeIsActive() && sensorDistance.getDistance(DistanceUnit.CM) > distance && (runtime.seconds() < timeOutS))   
          {        if (!opModeIsActive()){
                            return;}
                            
                            
           telemetry.addData("Distance", sensorDistance.getDistance(DistanceUnit.CM));
            
           // adjust relative speed based on heading error.
           error = getError(angle);
           steer = getSteer(error, P_DRIVE_COEFF);

           // if driving in reverse, the motor correction also needs to be reversed
           //if (distance < 0)
           //         steer *= -1.0;

           leftSpeed = speed - steer;
           rightSpeed = speed + steer;
               


           //Normalize speeds if either one exceeds +/- 1.0;
           max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
           if (max > 1.0)
                {
                leftSpeed /= max;
                rightSpeed /= max;
                }
 

                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(leftSpeed);
                motorRearRight.setPower(rightSpeed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);



        }
        
    }

public void encoderDriveLift(double speed,
                             double leftInches1, double leftInches2, double rightInches1,
                             double rightInches2, double liftPower, double liftInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
            newRearRightTarget = motorRearRight.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);
            newRearLeftTarget = motorRearLeft.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);
            newLiftTarget = motorLift.getCurrentPosition() + (int) (liftInches * (1140/(3.5 * 3.1415)));
    

            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorLift.setTargetPosition(newLiftTarget);
        
            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            

            // reset the timeout time and start motion.
            runtime.reset();
            motorFrontRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorRearRight.setPower(Math.abs(speed));
            motorRearLeft.setPower(Math.abs(speed));
            motorLift.setPower(Math.abs(liftPower));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);
            motorLift.setPower(0);


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
       
    }
    
    
    public void gyroStrafeLift ( double speed,  double distance,  double angle, double liftSpeed, double liftDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newLiftTarget;
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + moveCounts;
            newRearRightTarget = motorRearRight.getCurrentPosition() + moveCounts;
            newRearLeftTarget = motorRearLeft.getCurrentPosition() - moveCounts;
            newLiftTarget = motorLift.getCurrentPosition() + (int) (liftDistance * (1140/(3.5 * 3.1415)));

            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorLift.setTargetPosition(newLiftTarget);
            

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorRearRight.setPower(speed);
            motorRearLeft.setPower(speed);
            motorLift.setPower(liftSpeed);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontLeft.isBusy() && motorLift.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {


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
 
 
             
 
 
                motorFrontRight.setPower(leftSpeed);
                motorFrontLeft.setPower(speed);
                motorRearRight.setPower(speed);
                motorRearLeft.setPower(rightSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);
            motorLift.setPower(0);


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
    }


public void gyroDriveLift ( double speed,  double distance,  double angle, double liftSpeed, double liftDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newLiftTarget;
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + moveCounts;
            newRearRightTarget = motorRearRight.getCurrentPosition() + moveCounts;
            newRearLeftTarget = motorRearLeft.getCurrentPosition() + moveCounts;
            newLiftTarget = motorLift.getCurrentPosition() + (int) (liftDistance * (1140/(3.5 * 3.1415)));


            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorLift.setTargetPosition(newLiftTarget);
            

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorRearRight.setPower(speed);
            motorRearLeft.setPower(speed);
            motorLift.setPower(liftSpeed);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {


//progressive slow to a stable stopping speed when running above 70% power.
               // if(speed >.7 && motorFrontRight.getCurrentPosition()< (.75*newFrontRightTarget))
                //speed=.6;
                
            //    if(speed >.7 && motorFrontRight.getCurrentPosition()<(.85*newFrontRightTarget))
              //  speed=.5;





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
 
 
             
 
 
                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(leftSpeed);
                motorRearRight.setPower(rightSpeed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);
            motorLift.setPower(0);
            


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    public void gyroDriveDistanceRear ( double speed,  double angle, double distance, double timeOutS) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
    
       motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
     //set power level for strafe
        //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorFrontRight.setPower(-speed);
        motorFrontLeft.setPower(-speed);
        motorRearRight.setPower(-speed);
        motorRearLeft.setPower(-speed);

    // Ensure that the opmode is still active
     if (opModeIsActive()) 
     {
       runtime.reset();
            //insert alpha value
        while (opModeIsActive() && sensorDistanceBack.getDistance(DistanceUnit.CM) > distance && (runtime.seconds() < timeOutS)
)
          {        if (!opModeIsActive()){
                            return;}
                            
                            
           telemetry.addData("Distance", sensorDistanceBack.getDistance(DistanceUnit.CM));
            
           // adjust relative speed based on heading error.
           error = getError(angle);
           steer = getSteer(error, P_DRIVE_COEFF);

           // if driving in reverse, the motor correction also needs to be reversed
           //if (distance < 0)
           //         steer *= -1.0;

           leftSpeed = speed - steer;
           rightSpeed = speed + steer;
               


           //Normalize speeds if either one exceeds +/- 1.0;
           max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
           if (max > 1.0)
                {
                leftSpeed /= max;
                rightSpeed /= max;
                }
 

                motorFrontRight.setPower(-leftSpeed);
                motorFrontLeft.setPower(-rightSpeed);
                motorRearRight.setPower(-leftSpeed);
                motorRearLeft.setPower(-rightSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);



        }
        
    }

    public void gyroDriveLiftNoBrake ( double speed,  double distance,  double angle, double liftSpeed, double liftDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newLiftTarget;
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
            newFrontRightTarget = motorFrontRight.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + moveCounts;
            newRearRightTarget = motorRearRight.getCurrentPosition() + moveCounts;
            newRearLeftTarget = motorRearLeft.getCurrentPosition() + moveCounts;
            newLiftTarget = motorLift.getCurrentPosition() + (int) (liftDistance * (1140/(3.5 * 3.1415)));


            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorLift.setTargetPosition(newLiftTarget);
            

            // Turn On RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
         
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorFrontRight.setPower(speed);
            motorFrontLeft.setPower(speed);
            motorRearRight.setPower(speed);
            motorRearLeft.setPower(speed);
            motorLift.setPower(liftSpeed);
            

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearRight.isBusy()
                            && motorRearLeft.isBusy())) {


//progressive slow to a stable stopping speed when running above 70% power.
                if(speed >.7 && motorFrontRight.getCurrentPosition()< (.75*newFrontRightTarget))
                speed=.6;
                
                if(speed >.7 && motorFrontRight.getCurrentPosition()<(.85*newFrontRightTarget))
                speed=.5;





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
 
 
             
 
 
                motorFrontRight.setPower(rightSpeed);
                motorFrontLeft.setPower(leftSpeed);
                motorRearRight.setPower(rightSpeed);
                motorRearLeft.setPower(leftSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
          //  motorFrontRight.setPower(0);
            //motorFrontLeft.setPower(0);
            //motorRearRight.setPower(0);
            //motorRearLeft.setPower(0);
            //motorLift.setPower(0);
            


            // Turn off RUN_TO_POSITION
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
public void gyroStrafeColor ( double speed,  double angle, double alphaValue, double timeOutS) {

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
    
       motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
     //set power level for strafe
       // speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(-speed);
        motorRearRight.setPower(-speed);
        motorRearLeft.setPower(speed);

    // Ensure that the opmode is still active
     if (opModeIsActive()) 
     {
       runtime.reset();
            //insert alpha value
        while (opModeIsActive() && sensorColor.alpha() > alphaValue && (runtime.seconds() < timeOutS)
)
          {        if (!opModeIsActive())
                            return;
                            
                            
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        
           Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
           telemetry.addData("Alpha", sensorColor.alpha());
           telemetry.addData("Encoder", motorFrontRight.getCurrentPosition());
          
          


           // adjust relative speed based on heading error.
           error = getError(angle);
           steer = getSteer(error, P_DRIVE_COEFF);

           // if driving in reverse, the motor correction also needs to be reversed
           //if (distance < 0)
           //         steer *= -1.0;

           leftSpeed = speed - steer;
           rightSpeed = speed + steer;
               


           //Normalize speeds if either one exceeds +/- 1.0;
           max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
           if (max > 1.0)
                {
                leftSpeed /= max;
                rightSpeed /= max;
                }
 

                motorFrontRight.setPower(leftSpeed);
                motorFrontLeft.setPower(speed);
                motorRearRight.setPower(speed);
                motorRearLeft.setPower(rightSpeed);
                

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      motorFrontRight.getCurrentPosition(),
                        motorFrontLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorRearRight.setPower(0);
            motorRearLeft.setPower(0);



        }
        
    }
    


}


*/
}
