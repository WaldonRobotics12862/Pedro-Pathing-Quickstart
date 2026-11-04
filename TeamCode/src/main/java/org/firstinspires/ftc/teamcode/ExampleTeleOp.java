package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends OpMode {

    private static final double LAUNCHER_TICKS_PER_REV = 28.0;
    private static final double LAUNCHER_START_RPM = 950.0;
    private static final double LAUNCHER_RPM_STEP = 50.0;
    private static final double MAX_LAUNCHER_RPM = 6000.0;

    private Follower follower;
    private DcMotor intakeMotor;
    private DcMotorEx launcherMotor;

    // Intake toggle state
    private double intakePower = 0.0;
    private boolean prevA = false;
    private boolean prevB = false;

    // Launcher RPM state
    private double targetLauncherRpm = 0.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        // Hardware mapping using the exact names configured in biobuzz.xml
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakePower = 0.0;
        prevA = false;
        prevB = false;

        targetLauncherRpm = 0.0;
        prevDpadUp = false;
        prevDpadDown = false;
        prevDpadLeft = false;
        prevDpadRight = false;

        // Schedule continuous TeleOp commands with Ivy
        schedule(
                runDrive(),
                runIntake(),
                runLaunch()
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();

        double actualTicksPerSec = launcherMotor.getVelocity();
        double actualRpm = (actualTicksPerSec * 60.0) / LAUNCHER_TICKS_PER_REV;

        telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        telemetry.addData("Launcher Target RPM", "%.0f RPM", targetLauncherRpm);
        telemetry.addData("Launcher Actual RPM", "%.1f RPM (%.1f ticks/s)", actualRpm, actualTicksPerSec);
        telemetry.update();
    }

    private Command runDrive() {
        return infinite(() -> {
            follower.manual(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            follower.update();
        }).requiring(follower);
    }

    private Command runLaunch() {
        // D-Pad Up: starts at 950 RPM when stopped; subsequent presses increment by +50 RPM
        // D-Pad Down: decrements by -50 RPM on each press
        // D-Pad Left: resets launcher speed to 950 RPM
        // D-Pad Right: stops the launcher motor (0 RPM)
        return infinite(() -> {
            if (gamepad1.dpad_right && !prevDpadRight) {
                targetLauncherRpm = 0.0;
            } else if (gamepad1.dpad_left && !prevDpadLeft) {
                targetLauncherRpm = LAUNCHER_START_RPM;
            } else if (gamepad1.dpad_up && !prevDpadUp) {
                if (targetLauncherRpm == 0.0) {
                    targetLauncherRpm = LAUNCHER_START_RPM;
                } else {
                    targetLauncherRpm = Math.min(targetLauncherRpm + LAUNCHER_RPM_STEP, MAX_LAUNCHER_RPM);
                }
            } else if (gamepad1.dpad_down && !prevDpadDown) {
                targetLauncherRpm = Math.max(targetLauncherRpm - LAUNCHER_RPM_STEP, 0.0);
            }

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;

            // Convert RPM to ticks per second: (RPM * 28 ticks/rev) / 60 sec/min
            double targetTicksPerSec = (targetLauncherRpm * LAUNCHER_TICKS_PER_REV) / 60.0;
            launcherMotor.setVelocity(targetTicksPerSec);
        })
                .setEnd(endCondition -> launcherMotor.setVelocity(0))
                .requiring(launcherMotor);
    }

    private Command runIntake() {
        // Button A toggles forward intake (1.0 / 0.0)
        // Button B toggles reverse intake (-1.0 / 0.0)
        return infinite(() -> {
            if (gamepad1.a && !prevA) {
                intakePower = (intakePower > 0.0) ? 0.0 : 1.0;
            } else if (gamepad1.b && !prevB) {
                intakePower = (intakePower < 0.0) ? 0.0 : -1.0;
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;

            intakeMotor.setPower(intakePower);
        })
                .setEnd(endCondition -> intakeMotor.setPower(0))
                .requiring(intakeMotor);
    }
}
