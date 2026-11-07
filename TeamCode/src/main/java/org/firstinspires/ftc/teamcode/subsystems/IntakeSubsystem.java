package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem controlling the intake motor.
 *
 * The intake pulls balls into the robot for storage.
 * Motor runs forward to intake, reverse to eject, or stops.
 */
public class IntakeSubsystem {

    private final DcMotor intakeMotor;

    // Toggle state tracked internally for teleop commands
    private double intakePower = 0.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ── Low-level control ────────────────────────────────────────────────

    /** Set intake power directly (-1.0 to 1.0). */
    public void setPower(double power) {
        intakePower = power;
        intakeMotor.setPower(intakePower);
    }

    /** Stop the intake motor. */
    public void stop() {
        setPower(0.0);
    }

    /** @return current intake power */
    public double getPower() {
        return intakePower;
    }

    // ── Commands ─────────────────────────────────────────────────────────

    /**
     * Command that runs the intake forward at full power until interrupted.
     */
    public Command runForward() {
        return new CommandBuilder()
                .setStart(() -> setPower(1.0))
                .setEnd(endCondition -> stop())
                .requiring(intakeMotor);
    }

    /**
     * Command that runs the intake in reverse at full power until interrupted.
     */
    public Command runReverse() {
        return new CommandBuilder()
                .setStart(() -> setPower(-1.0))
                .setEnd(endCondition -> stop())
                .requiring(intakeMotor);
    }

    /**
     * Command that stops the intake immediately.
     */
    public Command stopCommand() {
        return Commands.instant(this::stop)
                .requiring(intakeMotor);
    }

    /**
     * Toggle-based teleop command: A toggles forward, B toggles reverse.
     * Runs continuously and manages edge-detection internally.
     *
     * @param aSupplier  supplies the current state of the "forward toggle" button
     * @param bSupplier  supplies the current state of the "reverse toggle" button
     * @return a continuous command suitable for scheduling during teleop
     */
    public Command teleopToggle(java.util.function.BooleanSupplier aSupplier,
                                java.util.function.BooleanSupplier bSupplier) {
        return new CommandBuilder()
                .setStart(() -> {
                    intakePower = 0.0;
                })
                .setExecute(new Runnable() {
                    private boolean prevA = false;
                    private boolean prevB = false;

                    @Override
                    public void run() {
                        boolean a = aSupplier.getAsBoolean();
                        boolean b = bSupplier.getAsBoolean();

                        if (a && !prevA) {
                            intakePower = (intakePower > 0.0) ? 0.0 : 1.0;
                        } else if (b && !prevB) {
                            intakePower = (intakePower < 0.0) ? 0.0 : -1.0;
                        }

                        prevA = a;
                        prevB = b;

                        intakeMotor.setPower(intakePower);
                    }
                })
                .setEnd(endCondition -> stop())
                .requiring(intakeMotor);
    }
}
