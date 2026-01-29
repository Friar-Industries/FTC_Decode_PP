/*
PRELOAD GREEN IN BOTTOM, PURPS ON TOP
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TestBotAuton", group = "A")
public class TestBotAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Integer pathState;
    private final Pose startPose = new Pose(104,136,Math.toRadians(90));
    private final Pose obeliskPose = new Pose(84,112,Math.toRadians(105));
    private final Pose shootPose = new Pose(85.65,84.28,Math.toRadians(45));
    private final Pose endPose = new Pose(95.88,73,Math.toRadians(45));
    private Path firstPath;
    private Path secondPath;
    private Path thirdPath;
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private Servo flick;
    private Servo sorter;
    private Limelight3A limelight;
    private final double FINGER_HOME = 1;
    private final double FINGER_ACTIVE = 0.1;
    private final double SORTER_LEFT = 0.06;
    private final double SORTER_HOME = 0.5;
    private final double SORTER_RIGHT = 0.93;
    private int[] pattern = {0,0,0};
    private String patternState = "PPG";

    public void buildPaths() {
        firstPath = new Path(new BezierLine(startPose,obeliskPose));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(),obeliskPose.getHeading());
        secondPath = new Path(new BezierLine(obeliskPose,shootPose));
        secondPath.setLinearHeadingInterpolation(obeliskPose.getHeading(),shootPose.getHeading());
        thirdPath = new Path(new BezierLine(shootPose,endPose));
        thirdPath.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    pattern = readObelisk();
                    
                    if (pattern[0] == 1) {
                        patternState = "GPP";
                    } else if (pattern[1] == 1) {
                        patternState = "PGP";
                    }

                    follower.followPath(secondPath,true);
                    setPathState(2);
                }
            case 2:
                if(!follower.isBusy()) {
                    shootBalls(patternState);

                    follower.followPath(thirdPath,true);
                    setPathState(3);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.addData("Pattern", Arrays.toString(pattern));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooterLeft = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight = hardwareMap.get(DcMotor.class, "shooter_right");
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sorter = hardwareMap.get(Servo.class, "sorter");
        sorter.setPosition(0.5);

        flick = hardwareMap.get(Servo.class, "flick");
        flick.setDirection(Servo.Direction.FORWARD);
        flick.setPosition(FINGER_HOME);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();

        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    public void flickBall() throws InterruptedException {
        flick.setPosition(FINGER_ACTIVE);
        sleep(1000);
        flick.setPosition(FINGER_HOME);
    }

    public void moveSorter(double pos) {
        sorter.setPosition(pos);
    }
    
    public void shootBalls(String state) throws InterruptedException {
        setShooterPower(0.75);
        int timeout = 1000;
        switch (state) {
            case "GPP":
                sleep(500);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_LEFT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_RIGHT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
            case "PGP":
                moveSorter(SORTER_LEFT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_HOME);
                sleep(timeout);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_RIGHT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
            case "PPG":
                moveSorter(SORTER_LEFT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_RIGHT);
                sleep(timeout);
                flickBall();
                sleep(timeout);
                moveSorter(SORTER_HOME);
                sleep(timeout);
                flickBall();
                sleep(timeout);
        }
    }
    public int[] readObelisk() {
        pattern = new int[]{0, 0, 0};
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                pattern[id-21] = 1;
            }
        }
        return pattern;
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
