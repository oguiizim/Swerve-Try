package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class DriveCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier vX, vY, headingAdjust;
    private final BooleanSupplier lookFront, lookBack, lookRight, lookLeft;
    private boolean resetHeading = false;

    public DriveCommand(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
            BooleanSupplier lookFront, BooleanSupplier lookBack, BooleanSupplier lookRight, BooleanSupplier lookLeft) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        this.lookFront = lookFront;
        this.lookBack = lookBack;
        this.lookRight = lookRight;
        this.lookLeft = lookLeft;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        resetHeading = true;
    }

    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;

        if (lookFront.getAsBoolean()) {
            headingY = -1;
        }
        if (lookBack.getAsBoolean()) {
            headingY = 1;
        }
        if (lookRight.getAsBoolean()) {
            headingX = 1;
        }
        if (lookLeft.getAsBoolean()) {
            headingX = -1;
        }

        if (resetHeading) {
            if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) == 0) {
                Rotation2d currentHeading = swerve.getHeading();

                headingX = currentHeading.getSin();
                headingY = currentHeading.getCos();
            }

            resetHeading = false;
        }

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(
                translation,
                swerve.getFieldVelocity(),
                swerve.getPose(),
                Dimensoes.LOOP_TIME,
                Dimensoes.ROBOT_MASS,
                List.of(Dimensoes.CHASSIS),
                swerve.getSwerveDriveConfiguration());

        if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
            resetHeading = true;
            swerve.drive(translation, (Tracao.TURN_CONSTANT * -headingAdjust.getAsDouble()));
        } else {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
