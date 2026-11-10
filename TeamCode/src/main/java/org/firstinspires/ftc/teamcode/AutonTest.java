package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.pedro.Constants.drivetrainConfig;
import static org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig;
import static org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@Autonomous (name = "Mr. Cox", group = "Autonomous")
public class AutonTest extends LinearOpMode {

    private Follower follower;

    // Subsystems
    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private LauncherSubsystem launcher;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.5, 9, 90);
    private final Pose point1 = poseFactory.of(11, 11, 90);
    private final Pose point1Control1 = poseFactory.of(30, 30, 0);
    private final Pose point1Control2 = poseFactory.of(11, 30, 0);
    private final Pose point2 = poseFactory.of(60, 110, -6.3402);
    private final Pose point2Control1 = poseFactory.of(15, 115, 0);
    private final Pose point2Control2 = poseFactory.of(45,34,0);
    private final Pose point2Control3 = poseFactory.of(15,85,0);
    private final Pose point2Control4 = poseFactory.of(15,110,0);

    private final Pose point3Start = poseFactory.of(60, 110, 0);
    private final Pose point3 = poseFactory.of(60, 130, -90);
    private final Pose point4 = poseFactory.of(15, 120, 180);
    private final Pose point4Control1 = poseFactory.of(60, 120, 0);
    private final Pose point4Control2 = poseFactory.of(30, 120, 0);

    // Autonomous routine
    public Command autoRoutine() {

        // Initialise subsystems
        intake = new IntakeSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);


        return parallel(
                launcher.runAtRpm(500),
                sequential(
                    waitMs(500),
                    parallel(
                        feeder.feedCommand(),
                        intake.runForward()
                    ),
                    waitMs(2000),
                    parallel(
                        intake.stopCommand(),
                        feeder.stopCommand()
                    ),
                    follow(follower, path1())
                    //follow(follower, path2()),
                    //follow(follower, path3()),
                    //follow(follower, path4())
                )
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
        return Paths.curve(start, point1Control1, point1Control2, point1).constant(point1);
    }

    public Path path2() {
        return Paths.curve(point1, point2Control1, point2Control2, point2Control3, point2Control4, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4).tangent();
    }
}