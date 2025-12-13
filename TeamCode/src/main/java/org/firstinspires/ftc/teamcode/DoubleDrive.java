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


@TeleOp(name="AutoPosTeleOp", group="A")
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
    private final Position cameraPosition = new Position(DistanceUnit.INCH,0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
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
    private final double BLUE_SHOOT = 0;
    private double prevX;

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
        pinpoint.resetPosAndIMU();

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(19.894,DistanceUnit.MM);
        pinpoint.setOffsets(-67,-121,DistanceUnit.MM);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        //initAprilTag();

    }
    @Override
    public void init_loop() {
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", blueTeam ? "Blue" : "Red");
        telemetry.addData("Marination", runtime.toString());

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        if (gamepad1.x) {
            blueTeam = true;
        } else if (gamepad1.b) {
            blueTeam = false;
        }

        binPos = blueTeam ? new Pose2D(DistanceUnit.INCH,72,0,AngleUnit.DEGREES,0) : new Pose2D(DistanceUnit.INCH,72,144,AngleUnit.DEGREES,0);
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

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
       builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()

    @Override
    public void stop() {
        //visionPortal.close();
        super.stop();
    }
}