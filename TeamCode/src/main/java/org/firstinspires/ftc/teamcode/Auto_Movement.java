/*

 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

public class Auto_Movement extends LinearOpMode {
    // Linear Movement
    double Currentx;
    double Currenty;
    double Currentr;
    DcMotor MotorFL;
    DcMotor MotorFR;
    DcMotor MotorBR;
    DcMotor MotorBL;
    // Lift/Arm/Claw
    static int timeToThree;
    static int timeToTwo;
    static int timeToOne;
    static int fullRotation = 360;
    Servo ClawL;
    Servo ClawR;
    CRServo ArmL;
    CRServo ArmR;
    DcMotor LiftL;
    DcMotor LiftR;
    GyroSensor armGyro;
    double baseAngle; //the angle that the arm starts out at. Horizontal and facing the front of the robot is zero. Increases counterclockwise like the unit circle
    // Camera
    Servo Cam;
    // Carousel
    DcMotor Carousel1;
    DcMotor Carousel2;
    @Override
    public void runOpMode() {}
    //This first group declares all the necessary information for that type of movement
    //Linear Movement
    public double Currentx(){return Currentx;}
    public double Currenty(){return Currenty;}
    public double Currentr(){return Currentr;}
    public void MotorFL(DcMotor Motor) {
        MotorFL = Motor;
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void MotorFR(DcMotor Motor) {
        MotorFR = Motor;
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void MotorBL(DcMotor Motor) {
        MotorBL = Motor;
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void MotorBR(DcMotor Motor) {
        MotorBR = Motor;
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //Arm/Lift/Claw
    public void ClawL(Servo servo){ ClawL = servo; }
    public void ClawR(Servo servo){ ClawR = servo; ClawR.setDirection(Servo.Direction.REVERSE);}
    public void ArmL(CRServo servo){ ArmL = servo; ArmL.setDirection(DcMotorSimple.Direction.REVERSE);}
    public void ArmR(CRServo servo){ ArmR = servo;}
    public void LiftL(DcMotor motor){LiftL= motor; LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void LiftR(DcMotor motor){LiftR= motor; LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void armGyro(GyroSensor _gyro){armGyro = _gyro; armGyro.calibrate();}
    //Camera
    public void CamServo(Servo servo){ Cam = servo; }
    //Carousel
    public void Carousel1(DcMotor Motor) {
        Carousel1 = Motor;
    }
    public void Carousel2(DcMotor Motor) {
        Carousel2 = Motor;
    }
    //end of group

    //This Move stops the movement after the set time
    public void Move(double x, double y, double r, long duration){
        Move(x,y,r);
        sleep(duration);
        StopMove();
    }
    //This Move requires the StopMove() to be ran to stop it
    public void Move(double x, double y, double r){
        Currentx = x;
        Currenty = y;
        Currentr = r;
        double PowerFL = (x - y - r);
        double PowerFR = (-x - y + r);
        double PowerBL = (-x - y - r);
        double PowerBR = (x - y + r);
        this.MotorFL.setPower(PowerFL);
        this.MotorFR.setPower(PowerFR);
        this.MotorBL.setPower(PowerBL);
        this.MotorBR.setPower(PowerBR);
    }
    //used to stop Movement at any time
    public void StopMove(){
        Move(0,0,0);
    }
    //used to make sure it does not go over 1
    public double PowerRatio (double x, double y, double r) {
        double Max = Math.abs(x) + Math.abs(y) + Math.abs(r);
        double d;
        if (Max > 1) { //this part makes it never go under 1 that way we can still go slow however it will keep any ratio of speed it has
            d = Max;
        } else {
            d = 1;
        }
        return d;
    }
    //Runs the Carousel at a set power
    public void Carousel1(double p){
        this.Carousel1.setPower(p);
    }
    public void Carousel2(double p){
        this.Carousel2.setPower(p);
    }

    //Rotates the Camera
    public void CamRotate(int position) {
        if (position == -2) {
            this.Cam.setPosition(0);
        }
        if (position == -1) {
            this.Cam.setPosition(.31);
        }
        if (position == 0) {
            this.Cam.setPosition(.385);
        }
        if (position == 1) {
            this.Cam.setPosition(.455);
        }
        if (position == 2) {
            this.Cam.setPosition(.6666);
        }

    }
    public void ArmOpen(){ClawL.setPosition(0);
        ClawR.setPosition(0);}
    public void ArmClose(){ClawL.setPosition(.2333);
        ClawR.setPosition(.2333);}
    public void Lift(int Level){
        if (Level == 0){// floor on the opposite side
            LiftL.setTargetPosition(-3 * fullRotation);
            LiftR.setTargetPosition(-3 * fullRotation);
            armRotation(60);
        }else
        if (Level == 1){
            LiftL.setTargetPosition(-3 * fullRotation);//this will be based on the data we get from testing the heights
            LiftR.setTargetPosition(-3 * fullRotation);//this will be based on the data we get from testing the heights
            armRotation(230);
        }else
        if (Level == 2){
            LiftL.setTargetPosition(-2 * fullRotation);//this will be based on the data we get from testing the heights
            LiftR.setTargetPosition(-2 * fullRotation);//this will be based on the data we get from testing the heights
            armRotation(230);
        }else
        if (Level == 3){// top level
            LiftL.setTargetPosition(-1 * fullRotation);//this will be based on the data we get from testing the heights
            LiftR.setTargetPosition(-1 * fullRotation);//this will be based on the data we get from testing the heights
            armRotation(230);
        }
        LiftL.setPower(0.1);
        LiftR.setPower(0.1);
    }
    public void armRotation(int _target) {
        int target_ = _target % 360;
        int currentRotation = armGyro.getHeading();
        if (currentRotation < target_) {
            ArmL.setPower(1);
            ArmR.setPower(1);
        } else if (currentRotation > target_) {
            ArmL.setPower(-1);
            ArmR.setPower(-1);
        } else {
            ArmL.setPower(0);
            ArmR.setPower(0);
        }
    }
}
