#include <vector>

#include "AutoStep.h"

class AutoSequence : public AutoStep {

    public:
        AutoSequence() {}

        void Init() {

            m_currentStep = m_steps.begin();
            m_lastStep = m_steps.end();
            m_currentStep->Init();
        }

        bool Execute() {

            if (m_currentStep->Execute()) {

                m_currentStep->Cleanup();
                if (m_currentStep != m_lastStep) {
                    
                    m_currentStep++;
                }
                else {

                    return true;
                }
            }
            return false;
        }

        void Cleanup() {}

        void AddStep(AutoStep &refStep) {

            m_steps.push_back(&refStep);
        }

    private:
        std::vector<AutoStep*> m_steps;
        std::vector<AutoStep*>::iterator m_currentStep;
        std::vector<AutoStep*>::iterator m_lastStep;
};