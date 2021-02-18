#ifndef AUTOSEQUENCE_H
#define AUTOSEQUENCE_H

#include <vector>

#include "AutoStep.h"

class AutoSequence : public AutoStep {

    public:
        AutoSequence() : AutoStep("AutoSequence") {}

        void _Init() {

            if (!m_steps.empty()) {
                m_currentStep = m_steps.begin();
                m_lastStep = std::prev(m_steps.end());
                (*m_currentStep)->Init();
                m_done = false;
            }
            else {

                m_done = true;
            }
        }

        bool _Execute() {

            if (!m_done) {
                
                if ((*m_currentStep)->Execute()) {

                    (*m_currentStep)->Cleanup();
                    if (m_currentStep != m_lastStep) {
                        
                        m_currentStep++;
                        (*m_currentStep)->Init();
                    }
                    else {

                        m_done = true;
                    }
                }
            }
            return m_done;
        }

        void _Cleanup() {}

        void AddStep(AutoStep* refStep) {

            m_steps.push_back(refStep);
        }

    private:
        std::vector<AutoStep*> m_steps;
        std::vector<AutoStep*>::iterator m_currentStep;
        std::vector<AutoStep*>::iterator m_lastStep;
        bool m_done;
};

#endif