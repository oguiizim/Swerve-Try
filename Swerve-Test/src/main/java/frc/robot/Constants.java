// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;

/**
 * Classe de constantes
 */
public final class Constants {

  public static final class Dimensoes {

    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô
    public static final double ROBOT_MASS = 34.65;

    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

    // Posições do centro de massa

    // Máxima aceleração e velocidade
    public static final double MAX_ACCE_AUTO = 4.72;
    public static final double MAX_VEL_AUTO = 4.72;

    // Diâmetro da roda do módulo
    public static final double wheelDiameterInMeters = 0.1016;

    // Redução para motor de acionamento e ângulo
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 21.42;

    // PPR do encoder interno NEO;
    public static final double pulsePerRotation = 1;

    // Fatores de conversão para motores de acionamento e ângulo
    public static final double driveConversion = SwerveMath.calculateMetersPerRotation(
        wheelDiameterInMeters,
        driveGearRatio,
        pulsePerRotation);
    public static final double angleConversion = SwerveMath.calculateDegreesPerSteeringRotation(
        angleGearRatio,
        pulsePerRotation);
  }

  public static final class Motors {
    public static final int frontIntake = 9;
    public static final int backIntake = 15;

    public static final int upperShooter = 9;
    public static final int lowerShooter = 11;
    public static final int conveyorShooter = 10;

    public static final int angleShooter1 = 12;
    public static final int angleShooter2 = 13;
  }

  public static final class PID {

    // PID para translação
    public static final PIDFConfig translationAutoPID = new PIDFConfig(4, 0, 0);
    // PID de rotação
    public static final PIDFConfig angleAutoPID = new PIDFConfig(4, 0, 0);
    // PID de rotação no proprio eixo
    public static final PIDFConfig anglePID = new PIDFConfig(2, 0, 0);

  }

  public static final class Controle {

    public static final int xboxControle = 0;
    public static final int controle2 = 1;

    public static final double DEADBAND = 0.1;

    public static final int xLeftAxis = 0;
    public static final int yLeftAxis = 1;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int xRightAxix = 4;
    public static final int yRightAxis = 5;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLB = 5;
    public static final int kRB = 6;
    public static final int kBack = 7;
    public static final int kStart = 8;
    public static final int kLeftAxisButton = 9;
    public static final int kRightAxisButton = 10;
  }

  public static final class Tracao {

    public static final boolean fieldRelative = true;
    public static final boolean isOpenLoop = false;
    // true para correção de aceleração
    public static final boolean accelCorrection = false;
    public static final double multiplicadorRotacional = 0.8;
    public static final double multiplicadorTranslacionalY = 0.9;
    public static final double multiplicadorTranslacionalX = 0.9;

    public static final double TURN_CONSTANT = 0.75;

    public static final double MAX_SPEED = 4.0;

    public static final double dt = 0.02;

    public static final double constantRotation = 4;
  }

  // Classe que guarda os nomes das trajetórias
  public static final class Trajetoria {

    public static final boolean ALIANCA = true; // Caso a aliança seja azul use false, se for vermelha use true
    // Auto Speaker: 4 notes
    public static final String TRY = "try";
    public static final String AUTO_MID_1 = "Auto Mid 1";
    public static final String AUTO_SOURCE_1 = "Auto Source 1";
    public static final String AUTO_SOURCE_2 = "Auto Source 2";
    // Auto Source: 2 notes
  }
}