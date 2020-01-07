package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.teamcode.Robot.*;


@Autonomous(name = "Gyro")
public class GyroTest extends LinearOpMode {
    boolean a1, b1, x1;

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        initIMU(this);

        waitForStart();
        double angle = 0;

        while (opModeIsActive()) {
            a1 = gamepad1.a;
            b1 = gamepad1.b;

            if (a1) {
                moveToAngle(angle + 90, 1);
            }
            if (b1) {
                moveToAngle(angle - 90, 1);
            }

        }
    }

}
