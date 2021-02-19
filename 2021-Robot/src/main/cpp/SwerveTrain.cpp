#include <math.h>

#include "SwerveTrain.h"
#include "VectorDouble.h"
#include "Launcher.h"
#include "Limelight.h"

void SwerveTrain::drive(const double rawX, const double rawY, const double rawZ, const bool precision, const bool record) {

    double x = -rawX;
    double y = -rawY;
    double z = rawZ; //* R_executionCapZion;

    if (record) {

        m_recorder->Record(rawX, rawY, rawZ);
    }
    else {

        m_recorder->Publish();
    }

    //To prevent controller drift, if the values of X, Y, and Z are inside of
    //deadzone, set them to 0.
    forceControllerXYZToZeroInDeadzone(x, y, z);

    //To prevent accidental turning, optimize Z to X and Y's magnitude.
    optimizeControllerXYToZ(x, y, z);

    //If the controller is in the total deadzone (entirely still)...
    if (getControllerInDeadzone(x, y, z)) {

        /*
        Go to the nearest zero position, take it as the new zero, and
        reset the angle coming off of the gyroscope to zero to allow
        setting of the next translation vector, as the translation
        vector is always refereced from "center" (the beginning of the
        turn) and updated based on how much that gyro angle changes.
        Due to this, it must be reset when not in movement to allow
        this behavior to occur...
        */
        setDriveSpeed(0);
        setSwerveSpeed(0);
        //else assumeNearestZeroPosition();
    }
    //Otherwise, go to the result vectors and use the magnitude to set the
    //speed of driving, and set each wheel's swerve position based on its
    //respective resulting vector.
    else {
        
        /*
        The translation vector is the "standard" vector - that is, if no
        rotation were applied, the robot would simply travel in the direction
        of this vector. In order to obtain this, we need the X and Y from the
        controller in addition to input from a gyroscope. This is due to the
        fact that pushing straight on the controller should always make it
        drive directly away from the operator, and simply driving "straight" at
        a 45* angle would make it drive away from the operator at 45*. This is
        true of any angle, so the gyro is needed to offset the vector described
        by X and Y. VectorDouble translationVector(0, 0);
        */
        //TODO: why inverted?
        VectorDouble translationVector(-x, y);

        //Get the navX yaw angle for computing the field-oriented angle
        double angle = navX->getYawFull();

        /*
        The rotational vectors are found by multiplying the controller's
        rotational axis [-1, 1] by the cosine of the wheel's RELATIVE yaw (the
        position we put the wheels in so that it can turn, with zero at the
        top) minus the number of degrees we are offset from 0.  Then, for the j
        value we do the same, except we use sine, for Y.  All angles passed as
        paramaters to cos() and sin() are converted to radians first.
        */
        VectorDouble frontRightRotationVector (

            z * cos((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180))
        );

        VectorDouble frontLeftRotationVector (

            z * cos((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearLeftRotationVector (

            z * cos((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearRightRotationVector (

            z * cos((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180))
        );

        /*
        And the vector we actually want to apply to the swerves is the sum of
        the two vectors - the vector that forms "straight" (translationVector)
        and the vector that forms strictly the rotation (rotationVector).
        */
        VectorDouble frontRightResultVector = translationVector + frontRightRotationVector;
        VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
        VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
        VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

        /*
        Here, all of the resulting vectors are converted into Nics so that they
        can be written to the swerve modules using assumeSwervePosition(). They
        were in degrees before to enable the use of common trigonomoetry. We
        get Nics from degrees by calling getSwerveRotatingPosition().
        */
        m_frontRight->assumeSwervePosition(m_frontRight->getStandardDegreeSwervePosition(frontRightResultVector, angle));
        m_frontLeft->assumeSwervePosition(m_frontLeft->getStandardDegreeSwervePosition(frontLeftResultVector, angle));
        m_rearLeft->assumeSwervePosition(m_rearLeft->getStandardDegreeSwervePosition(rearLeftResultVector, angle));
        m_rearRight->assumeSwervePosition(m_rearRight->getStandardDegreeSwervePosition(rearRightResultVector, angle));

        const double executionCap = precision ? R_executionCapZionPrecision : R_executionCapZion;
        m_frontRight->setDriveSpeed(frontRightResultVector.magnitude() * executionCap);
        m_frontLeft->setDriveSpeed(frontLeftResultVector.magnitude() * executionCap);
        m_rearLeft->setDriveSpeed(rearLeftResultVector.magnitude() * executionCap);
        m_rearRight->setDriveSpeed(rearRightResultVector.magnitude() * executionCap);
    }
}
double SwerveTrain::getClockwiseREVRotationsFromCenter(const VectorDouble &vector) {

    const double x = vector.i;
    const double y = vector.j;
    VectorDouble center(0, 1);
    VectorDouble current(x, y);

    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (magnitudeProduct == 0) {

        angleRad = 0;
    }

    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    double decimalTotalCircle = ((angleRad) / (2 * M_PI));
    return decimalTotalCircle * R_nicsConstant;
}
//TODO: Inline function documentation
double SwerveTrain::getStandardDegreeAngleFromCenter(const double &x, const double &y) {

    VectorDouble center(0, 1);
    VectorDouble current(x, y);

    const double dotProduct = center * current;
    const double magnitudeProduct = center.magnitude() * current.magnitude();
    const double cosineAngle = dotProduct / magnitudeProduct;
    double angleRad = acos(cosineAngle);
    if (magnitudeProduct == 0) {

        angleRad = 0;
    }

    if (x < 0) {

        angleRad = (2 * M_PI) - angleRad;
    }
    angleRad *= (180. / M_PI);
    return angleRad;
}
