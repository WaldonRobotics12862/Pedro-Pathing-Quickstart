package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends OpMode {

    private Follower follower;
    private DcMotor intakeMotor;
    private DcMotor launcherMotor;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        // Hardware mapping using the exact names configured in biobuzz.xml
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        launcherMotor = hardwareMap.get(DcMotor.class, "LauncherMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        telemetry.addData("LT (Intake Fwd)", gamepad1.left_trigger);
        telemetry.addData("LB (Intake Rev)", gamepad1.left_bumper);
        telemetry.addData("X (Intake Rev)", gamepad1.x);
        telemetry.addData("RT (Launcher)", gamepad1.right_trigger);
        telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        telemetry.addData("Launcher Motor Power", launcherMotor.getPower());
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
        // Logitech RT (Right Trigger): analog value 0.0 to 1.0
        return infinite(() -> launcherMotor.setPower(gamepad1.right_trigger))
                .setEnd(endCondition -> launcherMotor.setPower(0))
                .requiring(launcherMotor);
    }

    private Command runIntake() {
        // Logitech LB (Left Bumper): reverse intake (-1.0)
        // Logitech LT (Left Trigger): forward intake (0.0 to 1.0)
        return infinite(() -> {
            if (gamepad1.x) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(gamepad1.left_trigger);
            }
        })
                .setEnd(endCondition -> intakeMotor.setPower(0))
                .requiring(intakeMotor);
    }
}
