#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {

    public:
        static bool getControllerInDeadzone(const double x, const double y, const double z) {

            return abs(x) < R_deadzoneController && abs(y) < R_deadzoneController && abs(z) < R_deadzoneController;
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