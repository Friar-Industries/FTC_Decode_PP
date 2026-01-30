/*
TELEOP CONTROLS

Gamepad 1:

DURING INIT:

X - change team to blue
B - change team to red

DURING PLAY:

ERLIS:
Left stick - translate robot
Right stick - rotate robot
Dpad Up - high power
Dpad Down - low power
Dpad Left - low speed for parking
Dpad Right - high speed for driving
Left bumper - power down by .05
Right bumper - power up by .05
A - hold to turn robot to shooting angle

BRODY:
Dpad Up - Toggle flywheel
Dpad Down - Toggle intake
Dpad Left - Toggle feed servos
Right Bumper - Shoot ball
X - Cycle sorter left
B - Cycle sorter right
Y - Home sorter


Expansion -
Motor 0 front right
motor 1 back right
motor 2 shooter right
servo 0 flick
i2c 0 pinpoint

Control -
motor 0 shooter left
motor 1 front left
motor 2 back left
servo 1 sorter



*/

package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;


@Disabled
@Autonomous(name="LeaveAuton", group="A")
public class LeaveAuton extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final double power = 0.5;
    private double waitSeconds = 0;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) {
            waitSeconds += 0.5;
        } else if (gamepad1.dpadDownWasPressed()) {
            waitSeconds -= 0.5;
        }
    }

    @Override
    public void start() {
        runtime.reset();
        try {
            sleep((long) (waitSeconds * 1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        telemetry.addData("Elapsed Time", runtime.milliseconds() / 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        //visionPortal.close();
        super.stop();
    }
}