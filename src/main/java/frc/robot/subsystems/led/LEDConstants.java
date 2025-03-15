package frc.robot.subsystems.led;

import frc.robot.Constants;

/**
 * @brief Patterns taken from
 * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
 */
public class LEDConstants {
  public static final int DEFAULT = SolidColors.BLACK;

  public static final int[] PWM_PORTS =
      switch (Constants.getRobot()) {
        case COMP_BOT_2025 -> new int[] {0, 1};
        case T_SHIRT_CANNON_CHASSIS -> new int[] {0};
        case WOOD_BOT_TWO_2025 -> new int[] {0};
        default -> new int[] {};
      };

  public static final class FixedPalettePattern {
    public static final class Rainbow {
      public static final int RAINBOW_PALETTE = 1005;
      public static final int PARTY_PALETTE = 1015;
      public static final int OCEAN_PALETTE = 1025;
      public static final int LAVA_PALETTE = 1035;
      public static final int FOREST_PALETTE = 1045;
    }

    public static final class RainbowWithGlitter {
      public static final int RAINBOW_WITH_GLITTER = 1055;
    }

    public static final class Confetti {
      public static final int CONFETTI = 1065;
    }

    public static final class Shot {
      public static final int RED = 1075;
      public static final int BLUE = 1085;
      public static final int WHITE = 1095;
    }

    public static final class Sinelon {
      public static final int RAINBOW_PALETTE = 1105;
      public static final int PARTY_PALETTE = 1115;
      public static final int OCEAN_PALETTE = 1125;
      public static final int LAVA_PALETTE = 1135;
      public static final int FOREST_PALETTE = 1145;
    }

    public static final class BeatsPerMinute {
      public static final int RAINBOW_PALETTE = 1155;
      public static final int PARTY_PALETTE = 1165;
      public static final int OCEAN_PALETTE = 1175;
      public static final int LAVA_PALETTE = 1185;
      public static final int FOREST_PALETTE = 1195;
    }

    public static final class Fire {
      public static final int MEDIUM = 1205;
      public static final int LARGE = 1215;
    }

    public static final class Twinkles {
      public static final int RAINBOW_PALETTE = 1225;
      public static final int PARTY_PALETTE = 1235;
      public static final int OCEAN_PALETTE = 1245;
      public static final int LAVA_PALETTE = 1255;
      public static final int FOREST_PALETTE = 1265;
    }

    public static final class ColorWaves {
      public static final int RAINBOW_PALETTE = 1275;
      public static final int PARTY_PALETTE = 1285;
      public static final int OCEAN_PALETTE = 1295;
      public static final int LAVA_PALETTE = 1305;
      public static final int FOREST_PALETTE = 1315;
    }

    public static final class LarsonScanner {
      public static final int RED = 1325;
      public static final int GRAY = 1335;
    }

    public static final class LightChase {
      public static final int RED = 1345;
      public static final int BLUE = 1355;
      public static final int GRAY = 1365;
    }

    public static final class Heartbeat {
      public static final int RED = 1375;
      public static final int BLUE = 1385;
      public static final int WHITE = 1395;
      public static final int GRAY = 1405;
    }

    public static final class Breath {
      public static final int RED = 1415;
      public static final int BLUE = 1425;
      public static final int GRAY = 1435;
    }

    public static final class Strobe {
      public static final int RED = 1445;
      public static final int BLUE = 1455;
      public static final int GOLD = 1465;
      public static final int WHITE = 1475;
    }
  }

  public static final class Color1Pattern {
    public static final int END_TO_END_BLEND_TO_BLACK = 1485;
    public static final int LARSON_SCANNER = 1495;
    public static final int LIGHT_CHASE = 1505;
    public static final int HEARTBEAT_SLOW = 1515;
    public static final int HEARTBEAT_MEDIUM = 1525;
    public static final int HEARTBEAT_FAST = 1535;
    public static final int BREATH_SLOW = 1545;
    public static final int BREATH_FAST = 1555;
    public static final int SHOT = 1565;
    public static final int STROBE = 1575;
  }

  public static final class Color2Pattern {
    public static final int END_TO_END_BLEND_TO_BLACK = 1585;
    public static final int LARSON_SCANNER = 1595;
    public static final int LIGHT_CHASE = 1605;
    public static final int HEARTBEAT_SLOW = 1615;
    public static final int HEARTBEAT_MEDIUM = 1625;
    public static final int HEARTBEAT_FAST = 1635;
    public static final int BREATH_SLOW = 1645;
    public static final int BREATH_FAST = 1655;
    public static final int SHOT = 1665;
    public static final int STROBE = 1675;
  }

  public static final class Color1And2Pattern {
    public static final int SPARKLE_1_ON_2 = 1685;
    public static final int SPARKLE_2_ON_1 = 1695;
    public static final int COLOR_GRADIENT = 1705;
    public static final int BEATS_PER_MINUTE = 1715;
    public static final int END_TO_END_BLEND_1_TO_2 = 1725;
    public static final int END_TO_END_BLEND = 1735;
    public static final int COLOR_1_AND_2_NO_BLENDING = 1745;
    public static final int TWINKLES = 1755;
    public static final int COLOR_WAVES = 1765;
    public static final int SINELON = 1775;
  }

  public static final class SolidColors {
    public static final int HOT_PINK = 1785;
    public static final int DARK_RED = 1795;
    public static final int RED = 1805;
    public static final int RED_ORANGE = 1815;
    public static final int ORANGE = 1825;
    public static final int GOLD = 1835;
    public static final int YELLOW = 1845;
    public static final int LAWN_GREEN = 1855;
    public static final int LIME = 1865;
    public static final int DARK_GREEN = 1875;
    public static final int GREEN = 1885;
    public static final int BLUE_GREEN = 1895;
    public static final int AQUA = 1905;
    public static final int SKY_BLUE = 1915;
    public static final int DARK_BLUE = 1925;
    public static final int BLUE = 1935;
    public static final int BLUE_VIOLET = 1945;
    public static final int VIOLET = 1955;
    public static final int WHITE = 1965;
    public static final int GRAY = 1975;
    public static final int DARK_GRAY = 1985;
    public static final int BLACK = 1995;
  }
}
