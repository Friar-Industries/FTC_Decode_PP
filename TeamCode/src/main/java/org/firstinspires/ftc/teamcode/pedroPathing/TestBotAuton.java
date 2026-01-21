package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "TestBotAuton", group = "A")
public class TestBotAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Integer pathState;

    private final Pose startPose = new Pose(104,136,Math.toRadians(270));
    private final Pose shootPose = new Pose(85.65,84.28,Math.toRadians(45));
    private final Pose endPose = new Pose(95.88,73,Math.toRadians(45));
    private Path firstPath;
    private Path secondPath;

    public void buildPaths() {

        firstPath = new Path(new BezierLine(startPose,shootPose));
        firstPath.setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading());
        secondPath = new Path(new BezierLine(shootPose,endPose));
        secondPath.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                follower.followPath(secondPath);
                setPathState(2);
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
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
