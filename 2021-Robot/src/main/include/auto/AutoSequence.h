#ifndef AUTOSEQUENCE_H
#define AUTOSEQUENCE_H

#include <vector>

#include "AutoStep.h"

class AutoSequence : public AutoStep {

    public:
        AutoSequence(const bool &loop) : AutoStep("AutoSequence") {

            m_loop = loop;
        }

        void Init() {

            if (!m_steps.empty()) {
                m_currentStep = m_steps.begin();
                m_lastStep = m_steps.back();
                (*m_currentStep)->Init();
                m_done = false;
            }
            else {

                m_done = true;
            }
        }

        bool Execute() {

            if (!m_done) {
                
                // If the current step has finished
                if ((*m_currentStep)->Execute()) {

                    // If the step that just finished is the last step
                    if ((*m_currentStep) == m_lastStep) {
                        
                        // If we should loop
                        if (m_loop) {
                            
                            // Set the current step to the first step
                            m_currentStep = m_steps.begin();
                            // Initialize the first step
                            (*m_currentStep)->Init();
                        }
                        else {

                            // If we shouldn't loop, this AutoSequence is done
                            m_done = false;
                        }
                    }
                    else {

                        // Move on to the next step
                        m_currentStep++;
                        // Initialize the next step
                        (*m_currentStep)->Init();
                    }
                }
            }
            return m_done;
        }

        void AddStep(AutoStep* refStep) {

            m_steps.push_back(refStep);
        }

        void Reset() {

            m_steps.clear();
        }

    private:
        std::vector<AutoStep*> m_steps;
        std::vector<AutoStep*>::iterator m_currentStep;
        AutoStep* m_lastStep;
        bool m_done;
        bool m_loop;
};

#endif