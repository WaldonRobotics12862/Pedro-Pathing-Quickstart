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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@Autonomous(name = "RH_Simple_28", group = "Autonomous")
public class RH_Simple_28 extends LinearOpMode {

    private Follower follower;
    // Subsystems
    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private LauncherSubsystem launcher;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.6, 9, 90);
    private final Pose path1 = poseFactory.of(13.3691, 92.3521, 90);
    private final Pose path1Control1 = poseFactory.of(6.0623, 28.8216, 0);

    // Autonomous routine
    public Command autoRoutine() {
        // Initialise subsystems
        intake = new IntakeSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);

        return parallel(
                launcher.runAtRpm(1200),
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
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
