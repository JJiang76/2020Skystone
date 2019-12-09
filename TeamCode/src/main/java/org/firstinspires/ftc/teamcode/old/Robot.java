package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Robot {

    public Robot() {}

    final static int TICKS_PER_INCH = 90;

    final static double DEG90 = 22;//90 degree turn distance

    static DcMotor armboi;
    static DcMotor rightfront;
    static DcMotor leftfront;
    static DcMotor leftback;
    static DcMotor rightback;
    static DcMotor slide;
    static Servo grabright;
    static Servo grableft;
    static Servo blockgrabBlue;
    static Servo blockgrabRed;
    static Servo bord1;
    static Servo bord2;
    static Servo cap;


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Af+w9bD/////AAABmRAQ4PuIw0sdh/byTqA444Jo20dWZi+e6ZOHAsur5coDrAZ2k0LohL3kbR4Toa0yyYSOgCLXvIlm1ne8ZdjMTbVzEvsG0cfOrr3zHqV94jRsRa+sfeyeiTlEZFwz22a3eJX0CI5ga1JRfVaY8f3h1Kf6oxZbSF9rHraCjV1+egDARh4QmNWWHS0DKZi64hLwAu7P4NWsFFHN95eRdz3P7t4eQIZwX8vtAedGEkTM3V3tO8aYFcQ1MEPgHL+B+CTleFScXD8gjoMjCrVeZ4qNfkVda3bR3IUZSp8XaoL9GMy5irmgLJBNeo/H9qq3yFelSUIQMCZ1awAecpV6oHS3yAeaL8J+Bwe2/ZplShgiGPzS";
    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;


    public static void initMotors(OpMode opMode) {
        rightfront = opMode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opMode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opMode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opMode.hardwareMap.get(DcMotor.class, "rightback");
        armboi = opMode.hardwareMap.get(DcMotor.class, "armboi");
        slide = opMode.hardwareMap.get(DcMotor.class,"armboislides");

        //autonomous arms
        blockgrabBlue = opMode.hardwareMap.get(Servo.class, "servo2");
        blockgrabRed = opMode.hardwareMap.get(Servo.class, "servo4");
        //bottom two grabbers
        grabright = opMode.hardwareMap.get(Servo.class, "servo0");
        grableft = opMode.hardwareMap.get(Servo.class, "servo1");


        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);

        armboi.setDirection(DcMotor.Direction.REVERSE);

        armboi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors();
    }

    public static void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armboi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void initVuforia(OpMode opMode) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public static void initTfod(OpMode opMode) {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public static void forward(double dist, double speed) {
        move(speed, dist, dist, dist, dist);
    }
    public static void back(double dist, double speed) {
        move(speed, -dist, -dist, -dist, -dist);
    }
    public static void counter(double dist, double speed) {
        move(speed, -dist, -dist, dist, dist);
    }
    public static void clock(double dist, double speed){ move(speed, dist, dist, -dist, -dist); }
    public static void right(double dist, double speed) { move(speed, -dist, dist, dist, -dist); }
    public static void left(double dist, double speed) { move(speed, dist, -dist, -dist, dist); }


    public static void resetEncoders(){
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static double[] encoderValues(){
        double[] encoderArray = {rightfront.getCurrentPosition(), rightback.getCurrentPosition(), leftfront.getCurrentPosition(), leftback.getCurrentPosition()};
        return encoderArray;
    }

    public static void move(double speed, double distRF, double distRB, double distLF, double distLB) {
        resetEncoders();

        int posRF = (int)(distRF * TICKS_PER_INCH);
        int posRB = (int)(distRB * TICKS_PER_INCH);
        int posLF = (int)(distLF * TICKS_PER_INCH);
        int posLB = (int)(distLB * TICKS_PER_INCH);


        rightfront.setTargetPosition(posRF);
        rightback.setTargetPosition(posRB);
        leftfront.setTargetPosition(posLF);
        leftback.setTargetPosition(posLB);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setPower(speed);
        rightback.setPower(speed);
        leftfront.setPower(speed);
        leftback.setPower(speed);

        while(rightfront.isBusy() && rightback.isBusy() && leftfront.isBusy() && leftback.isBusy()) {

        }

        rightfront.setPower(0);
        rightback.setPower(0);
        leftfront.setPower(0);
        leftback.setPower(0);

    }

    public static void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);
    }


}
