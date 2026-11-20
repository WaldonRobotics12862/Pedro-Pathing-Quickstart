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
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@Autonomous(name = "LH_Simple_48", group = "Autonomous", preselectTeleOp = "ExampleTeleOp")
public class LH_Simple_48 extends LinearOpMode {

    private Follower follower;

    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private LauncherSubsystem launcher;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(61, 130, -90);
    private final Pose path1 = poseFactory.of(61, 70.3286, -90);
    private final Pose point2 = poseFactory.of(61, 119, -89.352);
    private final Pose point3Start = poseFactory.of(61, 119, -90);
    private final Pose point3 = poseFactory.of(18.0026, 120.8318, 180);
    private final Pose point3Control1 = poseFactory.of(42.0023, 118.8855, 0);

    // Autonomous routine
    public Command autoRoutine() {

        intake = new IntakeSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);

        return parallel(
                launcher.runAtRpm(1400),
                sequential(
                        follow(follower, path1()),
                        waitMs(18000),
//                        waitMs(5000),
                        follow(follower, path2()),
                        parallel(
                                feeder.feedCommand(),
                                intake.runForward()
                        ),
                        waitMs(5000),
                        parallel(
                                intake.stopCommand(),
                                feeder.stopCommand(),
                                launcher.stopCommand()
                        ),
                        follow(follower, path3())
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
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).tangent();
    }
}
