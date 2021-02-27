#ifndef ASYNCLOOP_H
#define ASYNCLOOP_H

#include "auto/AutoStep.h"

class AsyncLoop : public AutoStep {

    public:
        AsyncLoop() : AutoStep("AsyncLoop") {}

        void Init() {

            if (!m_steps.empty()) {

                for (unsigned int i = 0; i < m_steps.size(); ++i) {

                    m_steps[i]->Init();
                }
                m_done = false;
            }
            else {

                m_done = true;
            }
        }

        bool Execute() {

            bool allDone = true;
            for (unsigned int i = 0; i < m_steps.size(); ++i) {

                if (!m_steps[i]->Execute()) {

                    allDone = false;
                }
            }
            return allDone;
        }

        void AddStep(AutoStep* refStep) {

            m_steps.push_back(refStep);
        }
    
    private:
        std::vector<AutoStep*> m_steps;
        bool m_done;
};

#endif