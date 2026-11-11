package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Autonomous(name = "AutonTestSahas", group = "Autonomous")
public class AutonTestSahas extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.5, 9, 90);
    private final Pose point2Start = poseFactory.of(56.5, 9, 90);
    private final Pose point2 = poseFactory.of(9, 15, 90);
    private final Pose point3Start = poseFactory.of(9, 15, 90);
    private final Pose point3 = poseFactory.of(9, 12, 90);
    private final Pose point4Start = poseFactory.of(9, 12, 90);
    private final Pose point4 = poseFactory.of(62, 132, 90);
    private final Pose point4Control1 = poseFactory.of(36.9764, 46.2149, 0);
    private final Pose point4Control2 = poseFactory.of(1.5919, 75.9289, 0);
    private final Pose point4Control3 = poseFactory.of(26.8533, 89.4653, 0);
    private final Pose point4Control4 = poseFactory.of(42.3855, 96.5665, 0);
    private final Pose point5Start = poseFactory.of(62, 132, 90);
    private final Pose point5 = poseFactory.of(13.4101, 120.0921, 0);
    private final Pose point5Control1 = poseFactory.of(59.317, 113.2684, 0);
    private final Pose point5Control2 = poseFactory.of(18.6096, 120.4454, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4Control3, point4Control4, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5Control2, point5).linear(point5Start, point5);
    }
}
