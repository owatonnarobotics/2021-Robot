#include "RobotMap.h"

class AutoStep {
    
    public:
        virtual void Init() = 0;
        virtual bool Execute() = 0;
        virtual void Cleanup() = 0;
};