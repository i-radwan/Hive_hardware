#pragma once

class Utils {
public:

    static void swap (int* a, int*b) {
        int c = *a;
        *a = *b;
        *b = c;
    }

    static void swap (double* a, double*b) {
        double c = *a;
        *a = *b;
        *b = c;
    }

    static int twosToDecimal(int v) {
        int x = log2(v) + 1;

        for (int i = 0; i < x; i++)
           v = (v ^ (1 << i));

        return -(v + 1);
    }

    static double mapAngle(double a) {
        if (a > 180) {
            return a - 360;
        }

        if (a < -180) {
            return a + 360;
        }

        return a;
    }

    static double anglesSmallDifference(double a, double b) {
        return min(abs(a - b), 360 - abs(a - b)) % 360;
    }

    static double anglesLargeDifference(double a, double b) {
        return max(abs(a - b), 360 - abs(a - b)) % 360;
    }

    static double anglesAverageDifference(double a, double b) {
        return (int(Utils::anglesSmallDifference(a, b) +
                    Utils::anglesLargeDifference(a, b))) / 2;
    }

    static double degreesToRad(double a) {
        return a * M_PI / 180;
    };

    static int radToDegrees(double r) {
        return int(lround(r * 180 / M_PI) + 360) % 360;
    };

    static void angleResolution(double val, double& angleCos, double& angleSin) {
        angleCos = cos(val);
        angleSin = sin(val);
    };

    static double vecToAngle(double angleCos, double angleSin) {
        return atan2(angleSin, angleCos);
    };
};