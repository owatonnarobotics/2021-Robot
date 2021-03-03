/*
Public Function:
    void forceControllerXYZToZeroInDeadzone(const int&, const int&, const int&)
        If any of the passed X, Y, or Z values fall outside of their global
        deadzone, they will be set to 0. Otherwise, they are untouched.
    void optimizeControllerXYToZ(const double&, const double&, double &)
        Scales the value of Z with a propotion constant to the magnitude of
        X and Y. Makes rotation harder to incude as speed increases, which
        makes strafing with a controller much more reliable.
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {

    public:
        static bool getControllerInDeadzone(const double x, const double y, const double z) {

            return abs(x) < R_deadzoneController && abs(y) < R_deadzoneController && abs(z) < R_deadzoneControllerZ;
        }

        static void forceControllerXYZToZeroInDeadzone(double &x, double &y, double &z) {

            if (abs(x) < R_deadzoneController) {x = 0;}
            if (abs(y) < R_deadzoneController) {y = 0;}
            if (abs(z) < R_deadzoneControllerZ) {z = 0;}
        }

        static void optimizeControllerXYToZ(const double &x, const double &y, double &z) {

            double magnitudeXY = sqrt(x * x + y * y);
            double absZ = abs(z);
            double deadzoneAdjustmentZ = R_deadzoneControllerZ + .3 * magnitudeXY * R_deadzoneControllerZ;

            if (z > deadzoneAdjustmentZ) {

                z -= (deadzoneAdjustmentZ - R_deadzoneController);
            }
            else if (z < -deadzoneAdjustmentZ) {

                z += (deadzoneAdjustmentZ - R_deadzoneController);
            }
            if (absZ < deadzoneAdjustmentZ) {

                z = 0;
            }
        }
};

#endif