/*
This is the MainCode we will use for competition
All we need to do is set the position for the levels on the lift/arm
Edit Date: 12/9/2021
*/
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.lang.Math;


@TeleOp(name = "MainCode2")
public class MainCode2 extends CharlieRaymann {

    //Arm
    int Level;
    boolean Arm;
    Servo ClawL;
    Servo ClawR;
    CRServo ArmL;
    CRServo ArmR;
    DcMotor LiftL;
    DcMotor LiftR;
    //Movement
    float PostAuto = 0;
    DcMotor MotorFL;
    DcMotor MotorFR;
    DcMotor MotorBR;
    DcMotor MotorBL;
    double RobotY;
    double RobotX;
    double Dx;
    double Dy;
    double Dr;
    //Carousel
    DcMotor Carousel1;
    //IMU/Gyro
    BNO055IMU imu;
    float startingGyroX;
    float startingGyroY;
    float startingGyroZ;
    BNO055IMU.Parameters parameters;
    ElapsedTime time;


    @Override
    public void runOpMode() {

        /*
        This all sets up the imu
        */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "imuCalData.json";
        imu.initialize(parameters);
        Position initialPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
        Velocity initVelocity = new Velocity(DistanceUnit.MM, 0, 0, 0, 0);
        imu.startAccelerationIntegration(initialPosition, initVelocity, 100);
        time = new ElapsedTime();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        /*
        This sets up the motors and also sets the Gyo start position
        */
        ClawL = hardwareMap.servo.get("ClawL");
        ClawR = hardwareMap.servo.get("ClawR");
        ClawR.setDirection(Servo.Direction.REVERSE);
        ArmL = hardwareMap.crservo.get("ArmL");
        ArmR = hardwareMap.crservo.get("ArmR");
        LiftL = hardwareMap.dcMotor.get("LiftL");
        LiftR = hardwareMap.dcMotor.get("LiftR");
        Carousel1 = hardwareMap.dcMotor.get("Carousel1");
        MotorFL = hardwareMap.dcMotor.get("FrontLeft");
        MotorFR = hardwareMap.dcMotor.get("FrontRight");
        MotorBL = hardwareMap.dcMotor.get("BackLeft");
        MotorBR = hardwareMap.dcMotor.get("BackRight");
        Orientation o = imu.getAngularOrientation();
        startingGyroX = o.firstAngle;
        startingGyroY = o.secondAngle;
        startingGyroZ = o.thirdAngle;
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //gets personal preferences and uses them
                Arm();
                double Rad=xRotation();
                Movement();
                if (DriverMode()) { //This is where it will do driver POV or robot POV
                    RobotY = ((Dy * Math.cos(Rad)) + (Dx * Math.sin(Rad)));
                    RobotX = (-(Dy * Math.sin(Rad)) + (Dx * Math.cos(Rad)));
                } else{
                    RobotX = Dx;
                    RobotY = Dy;
                }
                Move(RobotX,RobotY,Dr);
                RotateCarousel();
            }
        }



    }
    public void Move (double x, double y, double r) {
        double ratio = PowerRatio(x,y,r);

        double PowerFL = ((-r*RotationSpeed()) + (y*DriveSpeed()) - (x*DriveSpeed()))/ratio;
        double PowerFR = ((r*RotationSpeed()) + (y*DriveSpeed()) + (x*DriveSpeed()))/ratio;
        double PowerBL = ((-r*RotationSpeed()) + (y*DriveSpeed()) +(x*DriveSpeed()))/ratio;
        double PowerBR = ((r*RotationSpeed()) + (y*DriveSpeed()) - (x*DriveSpeed()))/ratio;

        MotorFL.setPower(PowerFL);
        MotorFR.setPower(-PowerFR);
        MotorBL.setPower(PowerBL);
        MotorBR.setPower(-PowerBR);
    }
    public double PowerRatio (double x, double y, double r) {
        double Max = (Math.abs(x)*DriveSpeed()) + (Math.abs(y)*DriveSpeed()) + (Math.abs(r)*RotationSpeed());
        double d;
        if (Max > 1) { //this part makes it never go under 1 that way we can still go slow however it will keep any ratio of speed it has
            d = Max;
        } else {
            d = 1;
        }
        return d;
    }
    public double xRotation() {
        Orientation o = imu.getAngularOrientation();
        float x = o.firstAngle - startingGyroZ;
        return -x;

    }
    public void Start(){
        if(StartForward()){
            PostAuto = 0;
        }
        if(StartLeft()){
            PostAuto = (float) (Math.PI/2);
        }
        if(StartRight()){
            PostAuto = (float) ((3*Math.PI)/2);
        }
        if(StartBackwards()){
            PostAuto = (float) (Math.PI);
        }
    }
    public void RotateCarousel(){
        if(CarouselMan()) {
            double Car1 = Carousel1();
            Carousel1.setPower(Car1 * CarouselSpeed());
        } else{
            if(CarouselRed()){
                Carousel1.setPower(-CarouselSpeed());
            }
            if(CarouselBlue()){
                Carousel1.setPower(CarouselSpeed());
            }
            if(!CarouselBlue() && !CarouselRed()){
                Carousel1.setPower(0);
            }
        }
    }
    public void Arm(){
        double speedy;
        if (Open()){Arm = false;}
        if (Close()){Arm = true;}
        if (Arm && ArmToggle()){Arm = false;}
        if (!Arm && ArmToggle()){Arm = true;}
        if (Arm){ //Open
            ClawL.setPosition(.2333);
            ClawR.setPosition(.2333);
        }
        if (!Arm){ //Close
            ClawL.setPosition(0);
            ClawR.setPosition(0);
        }

        if(LevelMan()) {
            if (Levely() > .2 || Levely() < -.2) {
                speedy = Levely();
            } else {
                speedy = 0;
            }
            LiftL.setPower(speedy);
            LiftR.setPower(speedy);
            if (Levelr() != 0) {
                ArmL.setPower(-Levelr());
                ArmR.setPower(Levelr());
            } else {
                ArmL.setPower(-.05);
                ArmR.setPower(.05);
            }
        }
    }
    public void Movement(){
        Dx=movex();
        Dy=-movey(); //this is since the y stick is inverse
        Dr=mover();
        if (moveL()){Dx=-1;}
        if (moveR()){Dx=1;}
        if (!moveL() && !moveR() && movex()==0){Dx=0;}
        if (moveF()){Dy=-1;}
        if (moveB()){Dy=1;}
        if (!moveF() && !moveB() && movey()==0){Dy=0;}
    }
}








