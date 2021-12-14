/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Auto_Movement;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "Distance Test Auto")
public class Distance_Test_Auto extends OpMode {

    int r = 0;
    static int oneHeight = 7000;
    static int twoHeight = 3000;
    static int threeHeight = 0;
    static int positiveArmRotationPower = -1;
    public static int[] heights = new int[]{0, oneHeight, twoHeight, threeHeight};
    public static int[] armTimes = new int[]{0, 2300, 2250, 2150};
    static int timeToLevel = 2300;
    static int timeToHub = 400;
    static int timeToCarousel = 2000;
    static int timeToBase = 1000;
    static int caroTime = 500;
    static int targetHubRotation = -135;
    static int targetCaroRotation = -90;
    static float antiGravityPower = .06f;
    static DSensor.directions relevantX = DSensor.directions.W;
    static DSensor.directions relevantY = DSensor.directions.S;
    static boolean blueSide = true;
    double rotation;
    int targetX = -1;
    int targetY = -1;
    int targetRotation = 361;



    VuforiaLocalizer.Parameters parameters;
    //Vuforia
    //region Misc Vars
    boolean imageWasSeen;
    boolean isIntegrating;
    double lastTime;
    double lastLastTime;
    int lostCycles;
    //endregion
    //region IMU
    BNO055IMU imu;
    BNO055IMU.Parameters IMUParameters;
    //endregion

    //region Vuforia
    VuforiaLocalizer vuforia;
    WebcamName camera;
    VuforiaTrackables images;
    VuforiaTrackable image0;
    VuforiaTrackable image1;
    VuforiaTrackable image2;
    VuforiaTrackable image3;
    List<VuforiaTrackable> allTrackables;
    Boolean ThreeTrucks;
    Boolean CowTruck;
    Boolean RELIEF;
    Boolean Plane;
    //endregion

    //region Distance Sensors
    DSensor dsA; //This one should be defined as the one facing the blue wall
    DSensor dsB; //90 degrees counterclockwise from the last
    DSensor dsC; //90 degrees counterclockwise from the last
    DSensor dsD; //90 degrees counterclockwise from the last
    DSensor[] distanceSensors;
    //endregion

    //region Statics
    static int fieldWidth;
    static int tileWidth;
    static float[] imageSize;
    static double inchesToMeters;
    static double metersToInches;
    //endregion

    //region OpenGLMatrices
    OpenGLMatrix cameraLocation;
    OpenGLMatrix image1Location;
    OpenGLMatrix image2Location;
    OpenGLMatrix image3Location;
    OpenGLMatrix image4Location;
    OpenGLMatrix lastLocation = null;
    OpenGLMatrix lastLastLocation = null;
    //endregion
    // Tensor Flow
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQ1S+Z7/////AAABmRYP8wdQrkY/s7VOZVYtKtUuTeufR9VNeUbtkajCrXZGjKMUUziArTn1vgMEStfebA0pWziCf6B+qrUnDy0JhDEH27uwsPYAAS4Ok5ockuEzV22vTNAdxYsF9y3ji/61JKiaFn5sBTB6yh2FW7lK1wjpSaTjBiYX2DWHys8q+gSbGS/D5nR5s9MJ5I5aDIb6kRcCjAzUF/FNK3U8kyKqZgEgqP+QLt17bp4cBDAXWuz0EsVw6l1D8kNdeZmU6wYd2W6JqwXZk2DwoBi2KYngTnEo8hR39vg5DNMGxHjokjcUnpmszW2w/v9MlvZU5V/8AG96ImJ611IdThMh+RX5juwbDysiYavqMGj6PnIiNzg/";
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    // Data needed to calculate Distance
    double swfl = 5119.4; // Sensor Width * FocalLength in px
    double ObjectHeight = 1.875 * 25.4;
    double Sensory = 2.02;
    double FocalLength;
    double Num;
    double Den;
    int L=0;
    int Detect = 0;//number of ducks currently in sight
    //protected VuforiaTest1 Vuforia;
    DcMotor MotorFL;
    DcMotor MotorFR;
    DcMotor MotorBR;
    DcMotor MotorBL;
    Servo ClawL;
    Servo ClawR;
    CRServo ArmL;
    CRServo ArmR;
    DcMotor LiftL;
    DcMotor LiftR;
    // Camera
    Servo Cam;
    // Carousel
    DcMotor Carousel1;
    DcMotor Carousel2;
    long Level;
    protected Auto_Movement Movement;

    @Override
    public void init() {
        initVuforia();
        initTfod();
        //initVF();
        initDSensors();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IMUParameters = new BNO055IMU.Parameters();
        imu.initialize(IMUParameters);
        /**
         *
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
        Movement = new Auto_Movement();
        MotorFL = hardwareMap.dcMotor.get("FrontLeft");
        Movement.MotorFL(MotorFL);
        MotorFR = hardwareMap.dcMotor.get("FrontRight");
        Movement.MotorFR(MotorFR);
        MotorBL = hardwareMap.dcMotor.get("BackLeft");
        Movement.MotorBL(MotorBL);
        MotorBR = hardwareMap.dcMotor.get("BackRight");
        Movement.MotorBR(MotorBR);
        Cam = hardwareMap.servo.get("Cam");
        Movement.CamServo(Cam);
        ClawL = hardwareMap.servo.get("ClawL");
        Movement.ClawL(ClawL);
        ClawR = hardwareMap.servo.get("ClawR");
        Movement.ClawR(ClawR);
        Carousel1 = hardwareMap.get(DcMotor.class, "Carousel1");
        Movement.Carousel1(Carousel1);
        ArmL = hardwareMap.crservo.get("ArmL");
        ArmR = hardwareMap.crservo.get("ArmR");
        ArmL.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftL = hardwareMap.dcMotor.get("LiftL");
        LiftR = hardwareMap.dcMotor.get("LiftR");
        LiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftL.setTargetPosition(0);
        LiftR.setTargetPosition(0);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        Movement.ArmClose();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public double distance(double Height, double ImageHeight, double ImageWidth) {
        FocalLength = swfl / ImageWidth;
        Num = FocalLength * ObjectHeight * ImageHeight;
        Den = Height * Sensory;
        return (Num / Den) / 25.4;//25.4 is to convert to inches
    }

    public void initDSensors()
    {
        dsA = new DSensor(blueSide, new float[]{6, 0, 0}, 0, hardwareMap.get(DistanceSensor.class, "d1"));
        dsB = new DSensor(blueSide, new float[]{0, 6, 0}, 90, hardwareMap.get(DistanceSensor.class, "d2"));
        dsC = new DSensor(blueSide, new float[]{-6, 0, 0}, 180, hardwareMap.get(DistanceSensor.class, "d3"));
        dsD = new DSensor(blueSide, new float[]{0, -6, 0}, 270, hardwareMap.get(DistanceSensor.class, "d4"));
        distanceSensors = new DSensor[]{dsA, dsB, dsC, dsD};
    }

    public void loopDSensor(double _rotation)
    {
        DSensor.directions aDir = dsA.getDirection(_rotation); //has A calculate the direction it's facing
        DSensor.directions bDir = DSensor.incNinety(aDir); //Tells B its ninety degrees counterclockwise from whatever A calculated
        DSensor.directions cDir = DSensor.incNinety(bDir); //Tells C its ninety degrees counterclockwise from B
        DSensor.directions dDir = DSensor.incNinety(cDir); //Tells D its ninety degrees counterclockwise from C
        dsB.setDirection(bDir);
        dsC.setDirection(cDir);
        dsD.setDirection(dDir);
    }

    public void initVF() {
        //region Vuforia
        fieldWidth = 144;
        tileWidth = fieldWidth / 6;
        imageSize = new float[]{8.5f, 11}; //height, width
        inchesToMeters = 1 / 39.37;
        metersToInches = 39.37;
        cameraLocation = OpenGLMatrix.translation(0, 0, 0);
        image1Location = OpenGLMatrix.translation(-3 * tileWidth, .5f * tileWidth, 6.375f).rotated(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 90, 90, 0);
        image2Location = OpenGLMatrix.translation(-1.5f * tileWidth, -3 * tileWidth, 6.375f).rotated(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 180, 0, -90);
        image3Location = OpenGLMatrix.translation(1.5f * tileWidth, -3 * tileWidth, 6.375f).rotated(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 180, 0, -90);
        image4Location = OpenGLMatrix.translation(3 * tileWidth, .5f * tileWidth, 6.375f).rotated(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, -90, -90, 0);
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraName = camera;
        images = vuforia.loadTrackablesFromAsset("Test");
        image0 = images.get(0);
        image0.setName("Three Trucks"); //Three Trucks Image
        image1 = images.get(1);
        image1.setName("Cow Truck"); //Cow Truck Image
        image2 = images.get(2);
        image2.setName("R E L I E F"); //Relief Boat Image
        image3 = images.get(3);
        image3.setName("Plane"); //Plane Image
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(images);
        image3.setLocationFtcFieldFromTarget(image1Location);
        image2.setLocationFtcFieldFromTarget(image2Location);
        image0.setLocationFtcFieldFromTarget(image3Location);
        image1.setLocationFtcFieldFromTarget(image4Location);
        ((VuforiaTrackableDefaultListener) image0.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocation);
        ((VuforiaTrackableDefaultListener) image1.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocation);
        ((VuforiaTrackableDefaultListener) image2.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocation);
        ((VuforiaTrackableDefaultListener) image3.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocation);
        //endregion

        //region IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.calibrationDataFile = "IMUCalibrationData.json";
        imu.initialize(IMUParameters);
        //endregion
    }

    public void startVF() {
        lostCycles = 20;
        lastTime = 0;
        lastLastTime = 0;
        images.activate();
    }

    public void loopVF() {
        //region Seeing
        for (int i = 0; i < 4; i++) {

            VuforiaTrackable trackable = allTrackables.get(i);
            boolean isVisible = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();
            telemetry.addData(trackable.getName(), isVisible ? "Visible" : "Not Visible");//
            if (i == 0) {
                ThreeTrucks = isVisible;
            }
            if (i == 1) {
                CowTruck = isVisible;
            }
            if (i == 2) {
                RELIEF = isVisible;
            }
            if (i == 3) {
                Plane = isVisible;
            }
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLastLocation = lastLocation;
                lastLocation = robotLocationTransform;
                if (!imageWasSeen) {
                    imu.stopAccelerationIntegration();
                    isIntegrating = false;
                }
                imageWasSeen = true;
                break;
            } else if (isVisible) {
                break;
            }
            if (i == 3) {
                imageWasSeen = false;
            }
        }
        //endregion
        if (imageWasSeen) {
            lostCycles = 0;
        } else {
            lostCycles++;

        }
        lastLastTime = lastTime;
        lastTime = this.time;
    }

    String getPositionString() {
        if(lastLocation != null)
        {
            return lastLocation.formatAsTransform() + " - Camera";
        }
        else {
            return "unknown";
        }
    }

    public void CameraLoop() {
        //loopVF();
        if (tfod != null) {
            r++;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    Detect = 1;
                }
            }
        }
    }

    public int Detect() {
        int Seen = 0;
        if (Detect != 0) {
            Seen = 1;
        }
        return Seen;
    }

    public void Move(double x, double y, double r){
        double PowerFL = (x - y - r);
        double PowerFR = (-x - y + r);
        double PowerBL = (-x - y - r);
        double PowerBR = (x - y + r);
        this.MotorFL.setPower(PowerFL);
        this.MotorFR.setPower(PowerFR);
        this.MotorBL.setPower(PowerBL);
        this.MotorBR.setPower(PowerBR);
    }

    @Override
    public void loop(){
        rotation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        loopDSensor(rotation);
        double xPower = 0;
        double yPower = 0;
        double rPower = 0;
        int xDist = 0;
        int yDist = 0;
        for(DSensor d: distanceSensors)
        {
            DSensor.directions dir = d.getDirection();
            if(dir == relevantX)
            {
                xDist = (int) Math.round(d.getDistance(rotation));
            }else if(dir == relevantY)
            {
                yDist = (int) Math.round(d.getDistance(rotation));
            }
        }
        if(targetRotation < 361) {
            rPower = Integer.compare(targetRotation, (int) Math.round(rotation));
        }
        if(targetX > 0) {
            xPower = Integer.compare(targetX, xDist);//x < y = -1
        }
        if(targetY > 0) {
            yPower = Integer.compare(targetY, yDist);//x < y = -1
        }
        double cosR = Math.cos(Math.toRadians(rotation));
        double sinR = Math.sin(Math.toRadians(rotation));
        Move(xPower * cosR + yPower * sinR, yPower * cosR + xPower * sinR, rPower);
        LiftL.setPower(1);
        LiftR.setPower(1);
        telemetry.addData("Pos", LiftR.getCurrentPosition());
        telemetry.addData("Target", LiftR.getTargetPosition());
        telemetry.addData("Mode", LiftR.getMode());
        telemetry.addData("Position", getPositionString());
        telemetry.addData("Ducks Detected", Detect);
        telemetry.addData("D1", dsA.getValue() + ", " + dsA.getDirection());
        telemetry.addData("D2", dsB.getValue() + ", " + dsB.getDirection());
        telemetry.addData("D3", dsC.getValue() + ", " + dsC.getDirection());
        telemetry.addData("D4", dsD.getValue() + ", " + dsD.getDirection());
        telemetry.update();
    }

    void sleep(double _time)
    {
        double startTime = this.time;
        while(Double.compare(this.time - startTime, _time) == -1)
        {

        }
    }

    @Override
    public void start()
    {
        /*
        while(L==0 && r < 9){
            Movement.CamRotate(-1);
            sleep(2);
            CameraLoop();
            if (Detect!=0){
                L=1;
            }
            if (L==0){
                Movement.CamRotate(0);
                sleep(2);
                CameraLoop();
                if (Detect!=0){
                    L=2;
                }
            }
            if (L==0) {
                Movement.CamRotate(1);
                sleep(2);
                CameraLoop();
                if (Detect!=0) {
                    L = 3;
                }
            }
        }
        if(L == 0)
        {
            L = 2;
        }*/
        LiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveTo(20, -1, -1, 2);
        moveTo(-1, -1, 90, 2);
        requestOpModeStop();
        //Movement.Lift(Detect());
        //move to alliance hub

        /*Movement.Move(0,1,0,400);//Move up to it
        Movement.Move(0,0,1,450);//Rotate to face it
        Movement.Move(0,.5,0);// Back away
        if(L != 3) {
            LiftL.setTargetPosition(heights[L]);
            LiftR.setTargetPosition(heights[L] - 1000);
        }
        LiftL.setPower(1);//Change the lift height
        LiftR.setPower(1);
        sleep(100);
        ArmL.setPower(positiveArmRotationPower);//Rotate the arm
        ArmR.setPower(positiveArmRotationPower);
        sleep(150);
        Movement.StopMove();
        sleep(armTimes[L] - 250);//Stop rotating the arm based on the level
        ArmL.setPower(antiGravityPower);//Hold the arm position for a little while
        ArmR.setPower(antiGravityPower);
        sleep(4250 - armTimes[L]);
        Movement.Move(0,-.5,0,370);//Move into the hub
        Movement.ArmOpen();
        Movement.ArmOpen();//Open the claw
        ClawL.setPosition(0);
        ClawR.setPosition(0);
        //move to carousel
        sleep(500);
        Movement.Move(-.5,0,0,1350);
        Movement.Move(0,0,-.5,250);//Move to the carousel
        Movement.Move(0,.5,0,900);
        Movement.Move(.1,.1,0,800);
        Movement.Carousel1(1);//Spin the carousel
        sleep(3000);
        Movement.Carousel1(0);//Stop spinning the carousel
        Movement.Move(Math.sin(Math.toRadians(rotation)), Math.cos(Math.toRadians(rotation)), 0, 400);//Move to the shipping hub
        stop();
        //Movement.Lift(Detect());


        //move to alliance hub
        //Movement.Move(0,1,0);

        /*sleep(timeToHub - timeToLevel);
        while(rotation > targetHubRotation)
        {
            Movement.Move(0, 0, -1);
        }
        Movement.StopMove();
        Movement.ArmOpen();
        //move to carousel
        Movement.Move(.5,-.5,0);
        sleep(100);
        ArmL.setPower(-positiveArmRotationPower);
        ArmR.setPower(-positiveArmRotationPower);
        sleep(100);
        ArmL.setPower(0);
        ArmR.setPower(0);
        sleep(timeToCarousel - 200);
        Movement.StopMove();
        Movement.Carousel1(1);
        sleep(caroTime);
        Movement.Carousel1(0);
        while (rotation > -180 && rotation < 270)
        {
            Movement.Move(0, 0, -1);
        }
        Movement.StopMove();
        Movement.Move(-1, 0, 0, timeToBase);*/
               /* Movement.Move(0,0,-1,100);//need to get exact timing and what direction to rotate
                Movement.Move(0,1,0,100);//need to get exact timing
                Movement.Carousel1(1);
                Movement.Carousel1(0);
                //move to storage unit
                Movement.Move(0,1,0,100);//need to get exact timing
              */  //or move to warehouse
                /*
                Movement.Move(0,0,1,100);//need to get exact timing
                Movement.Move(0,1,0,1000);//need to get exact timing
                 */
    }
    @Override
    public void stop()
    {
        telemetry.addLine("L"+String.valueOf(L));
        telemetry.addLine("Detect"+String.valueOf(Detect));
        telemetry.update();
    }

    public void moveTo(int _x, int _y, int _r, double _t)
    {
        targetX = _x;
        targetY = _y;
        targetRotation = _r;
        sleep(_t);
    }
}


