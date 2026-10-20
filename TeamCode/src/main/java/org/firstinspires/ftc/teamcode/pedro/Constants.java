package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("Front Left");
        c.frontRightName.set("Front Right");
        c.backLeftName.set("Back Left");
        c.backRightName.set("Back Right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        //c.zeroPowerBehavior.set(DcMotor.ZeroPowerBehavior.BRAKE);
    });
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(1.1573217149478563);
        c.angularScalar.set(0.991521474694527);
        c.offset.set(new Pose(-0.30637726070374016, 0.28234767162893704));
        c.linearUnit.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.28472178568123935);
                Controller secondaryTranslationalForward = Controller.proportional(0.1051970919029877);
                Controller primaryTranslationalLateral = Controller.proportional(0.4734797072099236);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17493810020328204);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012905243359594405));
                c.brake.set(Controller.proportionalFeedforward(0.010969456855655244));

                c.headingFeedback.set(Controller.proportional(5.054114053918472));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.46629179474781246, -0.14460631536791002));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07, 0.0015));
                //c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.005));
//                c.linearBrakeCoefficients.set(Matrix.diag(3.3385980136893556, 0.2961508809504582));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.041969959953436574, 0.003123070283624565));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.0007));

                c.maxAchievableForwardVelocity.set(85.13888546356549);
                c.maxAchievableStrafeVelocity.set(70.43958187156503);
                c.naturalForwardDeceleration.set(24.311882104890866);
                c.naturalStrafeDeceleration.set(63.42012739296694);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}