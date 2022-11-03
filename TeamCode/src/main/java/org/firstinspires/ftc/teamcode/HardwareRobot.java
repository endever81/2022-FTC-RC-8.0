package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareRobot {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor lift = null;
   // public DcMotor spinner = null;
   // public CRServo leftintake = null;
   // public CRServo rightintake = null;
    public Servo claw;
    public Servo rotator;


    //public Servo grabber = null;

    HardwareMap hwMap = null;
   
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot (){
    }
   
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap; //saves a reference Hardware Map
       
        leftFrontDrive = hwMap.get(DcMotor.class, "motor_left_front");
        rightFrontDrive = hwMap.get(DcMotor.class, "motor_right_front");
        leftRearDrive = hwMap.get(DcMotor.class, "motor_left_rear");
        rightRearDrive = hwMap.get(DcMotor.class, "motor_right_rear");
        lift = hwMap.get(DcMotor.class, "lift_motor");
        //spinner = hwMap.get(DcMotor.class, "motor_spinner");
       
       //leftintake = hwMap.get(CRServo.class, "left_intake");
       //rightintake = hwMap.get(CRServo.class, "right_intake");
       claw = hwMap.get(Servo.class, "claw_servo");
       rotator = hwMap.get(Servo.class, "rotator_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        //rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        //spinner.setPower(0);
        claw.setPosition(1);
        rotator.setPosition(.4);
        //leftintake.setPower(0);
        //rightintake.setPower(0);
    }

}

