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
X -

*/

package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Vector;


@TeleOp(name="DoubleDrive", group="A")
public class DoubleDrive extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private double power = 0.9;
    private Pose2D binPos;
    private final double MIN_POWER = 0.75;
    private final double MAX_POWER = 0.9;
    private double speed = 1;
    private final double LOW_SPEED = 0.6;
    private final double HIGH_SPEED = 1;
    private boolean intakeSpinning = false;
    private boolean spinning = false;
    private boolean blueTeam = true; // TRUE = BLUE TEAM, FALSE = RED TEAM
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private Servo intakeLeft;
    private Servo intakeRight;
    private Servo feed;
    private Servo flick;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private HuskyLens huskyLens;
    private GoBildaPinpointDriver pinpoint;

    public int[] readObelisk() {
        int[] pattern = {0,0,0};
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            pattern[block.id - 1] = 1;
        }
        return pattern;
    }

    public double getAprilTagPosX() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            return block.x - 160;
        }
        return 0;
    }

    public double getTargetYaw(Pose2D bin) {
        Pose2D robotPos = pinpoint.getPosition();
        double distX = bin.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double distY = bin.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);
        return Math.tan(distY/distX);
    }

    @Override
    public void init() {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight = hardwareMap.get(DcMotor.class, "shooter_right");
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        huskyLens = hardwareMap.get(HuskyLens.class, "camera");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft = hardwareMap.get(Servo.class, "intake_left");
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        intakeRight = hardwareMap.get(Servo.class, "intake_right");
        intakeRight.setDirection(Servo.Direction.FORWARD);

        feed = hardwareMap.get(Servo.class, "feed");
        feed.setDirection(Servo.Direction.REVERSE);
        flick = hardwareMap.get(Servo.class, "flick");
        flick.setDirection(Servo.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-73.51,-165.8,DistanceUnit.MM);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    @Override
    public void init_loop() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", blueTeam ? "Blue" : "Red");
        telemetry.addData("Marination", runtime.toString());
        telemetry.update();

        if (gamepad1.x) {
            blueTeam = true;
        } else if (gamepad1.b) {
            blueTeam = false;
        }

        //binPos = blueTeam ? new Pose2D(DistanceUnit.INCH,72,0,AngleUnit.DEGREES,0) : new Pose2D(DistanceUnit.INCH,72,144,AngleUnit.DEGREES,0);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double max;

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if (gamepad1.a) {
            yaw = getAprilTagPosX()/160;
        }

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= speed;
        frontRightPower *= speed;
        backLeftPower *= speed;
        backRightPower *= speed;

        if (gamepad1.bWasPressed()) {
            pinpoint.resetPosAndIMU();
        }


        // CHANGE SHOOTER POWER MANUALLY
        /*
        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            power += 0.01;
        } else if (gamepad1.dpadLeftWasPressed()) {
            power -= 0.05;
        } else if (gamepad1.dpadRightWasPressed()) {
            power += 0.05;
        } */

        // Set power

        if (gamepad1.dpadUpWasPressed()) {
            power = MAX_POWER;
        } else if (gamepad1.dpadDownWasPressed()) {
            power = MIN_POWER;
        }

        // Set power
        if (gamepad1.dpadRightWasPressed()) {
            speed = HIGH_SPEED;
        } else if (gamepad1.dpadLeftWasPressed()) {
            speed = LOW_SPEED;
        }

        // SPIN SHOOTER
        if (gamepad2.dpadUpWasPressed()) {
            spinning = !spinning;
        }

        if (gamepad2.xWasPressed()) {
            intakeSpinning = !intakeSpinning;
        }

        intakeLeft.setPosition(intakeSpinning ? 1 : 0.5);
        intakeRight.setPosition(intakeSpinning ? 1 : 0.5);
        feed.setPosition(intakeSpinning ? 1 : 0.5);

        shooterLeft.setPower(spinning ? power : 0);
        shooterRight.setPower(spinning ? power : 0);

        // FLICK BALL INTO SHOOTER
        if (gamepad2.y) {
            flick.setPosition(1);
        } else {
            flick.setPosition(0);
        }

        // CHANGE POWER MANUALLY BY .05
        if (gamepad1.leftBumperWasPressed()) {
            power -= .05;
        } else if (gamepad1.rightBumperWasPressed()) {
            power += .05;
        }

        // canLift = gamepad1.left_bumper && gamepad2.left_bumper;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        pinpoint.update();

        telemetry.addData("Power", power);
        telemetry.addData("Speed",speed);
        telemetry.addData("X",pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y",pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Encoder X",pinpoint.getEncoderX());
        telemetry.addData("Encoder Y",pinpoint.getEncoderY());
        telemetry.addData("Heading",pinpoint.getHeading(AngleUnit.DEGREES));
        //telemetryAprilTag();
        telemetry.addData("Elapsed Time", runtime.milliseconds() / 1000);
        telemetry.update();
    }

    @Override
    public void stop() {
        //visionPortal.close();
        super.stop();
    }
}