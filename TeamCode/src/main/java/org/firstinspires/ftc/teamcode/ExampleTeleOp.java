    package org.firstinspires.ftc.teamcode;

    import static com.pedropathing.ivy.Scheduler.schedule;
    import static com.pedropathing.ivy.commands.Commands.infinite;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.pedropathing.drivetrain.DrivePowers;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.follower.ManualDrive;
    import com.pedropathing.ivy.Command;
    import com.pedropathing.ivy.Scheduler;
    import com.pedropathing.math.Pose;
    import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.pedro.Constants;
    import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
    import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
    import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    import com.qualcomm.robotcore.hardware.DigitalChannel;


    @TeleOp(name = "Example TeleOp")
    public class ExampleTeleOp extends OpMode {

        private Follower follower;
        private SparkFunOTOS otos;

        // Subsystems
        private IntakeSubsystem intake;
        private FeederSubsystem feeder;
        private LauncherSubsystem launcher;

        //Logger
        private FtcDashboard dashboard;
        private TelemetryPacket packet;

        private DigitalChannel left_eye=null;
        private DigitalChannel right_eye=null;


        @Override
        public void init() {

            dashboard = FtcDashboard.getInstance();
            packet = new TelemetryPacket();
            Scheduler.reset();

            // Forward all telemetry to FTC Dashboard and AdvantageScope
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            follower = Constants.create(hardwareMap);
            otos = hardwareMap.get(SparkFunOTOS.class, "otos");

            // Initialise subsystems
            intake = new IntakeSubsystem(hardwareMap);
            feeder = new FeederSubsystem(hardwareMap);
            launcher = new LauncherSubsystem(hardwareMap);

            right_eye = hardwareMap.get(DigitalChannel.class, "right_eye");
            left_eye = hardwareMap.get(DigitalChannel.class, "left_eye");
            right_eye.setMode(DigitalChannel.Mode.OUTPUT);
            left_eye.setMode(DigitalChannel.Mode.OUTPUT);
            right_eye.setState(false);
            left_eye.setState(false);

            // Schedule continuous TeleOp commands with Ivy
            schedule(
                    runDrive(),
                    intake.teleopToggle(() -> gamepad2.a, () -> gamepad2.b),
                    launcher.teleopDpadControl(
                            () -> gamepad2.x,
                            () -> gamepad2.y,
                            () -> gamepad2.dpad_up,
                            () -> gamepad2.dpad_down,
                            () -> gamepad2.dpad_left,
                            () -> gamepad2.dpad_right
                    ),
                    feeder.teleopHoldToFeed(() -> gamepad2.left_bumper, () -> gamepad2.right_bumper)
            );
        }

        @Override
        public void loop() {
            packet = new TelemetryPacket();
            Scheduler.execute();

            right_eye.setState(false);
            left_eye.setState(false);

            // Add your custom robot data
            packet.put("Intake/Power", intake.getPower());
            packet.put("Launcher/Target RPM", launcher.getTargetRpm());
            packet.put("Launcher/Actual RPM", launcher.getActualRpm());
            packet.put("Launcher/Actual TicksPerSec", launcher.getActualTicksPerSec());

            // Subsystem telemetry (numeric values for AdvantageScope line graph plotting)
            telemetry.addData("Intake/Power", intake.getPower());
            telemetry.addData("Launcher/Target RPM", launcher.getTargetRpm());
            telemetry.addData("Launcher/Actual RPM", launcher.getActualRpm());
            telemetry.addData("Launcher/Actual TicksPerSec", launcher.getActualTicksPerSec());

            dashboard.sendTelemetryPacket(packet);


            // Robot pose telemetry (for AdvantageScope 2D/3D field visualization)
            if (follower != null && follower.pose() != null) {
                telemetry.addData("Robot/X", follower.pose().x());
                telemetry.addData("Robot/Y", follower.pose().y());
                telemetry.addData("Robot/Heading Deg", Math.toDegrees(follower.pose().heading()));
            }

            telemetry.update();
        }

        private Command runDrive() {
            return infinite(() -> {
                if (gamepad1.a) {
                    Pose currentPose = follower.pose();
                    follower.setPose(new Pose(currentPose.x(),currentPose.y(),0));
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
