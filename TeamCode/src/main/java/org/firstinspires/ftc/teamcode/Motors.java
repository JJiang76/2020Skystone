package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors {
    public static DcMotor rightfront;
    public static DcMotor leftfront;
    public static DcMotor leftback;
    public static DcMotor rightback;


    public static void initMotors(OpMode opmode) {
        rightfront = opmode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opmode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opmode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opmode.hardwareMap.get(DcMotor.class, "rightback");
    }
}
