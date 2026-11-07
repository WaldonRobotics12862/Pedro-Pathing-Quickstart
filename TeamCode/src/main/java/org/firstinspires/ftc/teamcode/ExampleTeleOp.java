package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends OpMode {

    private Follower follower;
    private SparkFunOTOS otos;

    // Subsystems
    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private LauncherSubsystem launcher;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        // Initialise subsystems
        intake = new IntakeSubsystem(hardwareMap);
        feeder = new FeederSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);

        // Schedule continuous TeleOp commands with Ivy
        schedule(
                runDrive(),
                intake.teleopToggle(() -> gamepad2.a, () -> gamepad2.b),
                launcher.teleopDpadControl(
                        () -> gamepad2.dpad_up,
                        () -> gamepad2.dpad_down,
                        () -> gamepad2.dpad_left,
                        () -> gamepad2.dpad_right
                ),
                feeder.teleopHoldToFeed(() -> gamepad2.left_bumper)
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addData("Intake Motor Power", intake.getPower());
        telemetry.addData("Launcher Target RPM", "%.0f RPM", launcher.getTargetRpm());
        telemetry.addData("Launcher Actual RPM", "%.1f RPM (%.1f ticks/s)",
                launcher.getActualRpm(), launcher.getActualTicksPerSec());
        telemetry.update();
    }

    private Command runDrive() {
        return infinite(() -> {
            if (gamepad1.a) {
                otos.calibrateImu();
            }

            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);
            follower.update();
        }).requiring(follower);
    }
}
