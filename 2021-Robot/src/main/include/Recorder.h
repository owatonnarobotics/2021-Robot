#ifndef RECORDER_H
#define RECORDER_H

#include <sstream>
#include <string>
#include <frc/SmartDashboard/SmartDashboard.h>

class Recorder {

    public:
        Recorder() {}

        void Record(const double x, const double y, const double z) {

            m_log << x << '-' << y << '-' << z << '-'; 
            SetStatus("Recording in progress...");
        }

        void Publish() {

            std::string rawString = m_log.str();
            if (rawString == "") {

                SetStatus("Standing by...");
            }
            else {

                std::string finalString = rawString;
                char currentChar;
                int posFirstUnique = -1;
                for (int i = 0; i < (int)(rawString.length()) && posFirstUnique == -1; ++i) {

                    currentChar = rawString.at(i);
                    if (currentChar != '0' && currentChar != '.' && currentChar != '-') {

                        posFirstUnique = i;
                    }
                }
                if (posFirstUnique != -1) {

                    if (posFirstUnique != 0) {

                        int posLatestHyphen = -1;
                        for (int i = posFirstUnique - 1; i >= 0 && posLatestHyphen == -1; --i) {

                            if (rawString.at(i) == '-') {

                                posLatestHyphen = i;
                            }
                        }
                        if (posLatestHyphen != -1) {

                            finalString = rawString.substr(posLatestHyphen + 1, rawString.length() - posLatestHyphen - 1);
                        }
                    }
                }
                else {

                    finalString = "";
                }
                
                SetStatus(finalString);
                m_log.str("");
            }
        }

        void SetStatus(std::string status) {

            frc::SmartDashboard::PutString("Recorder::m_log", status);
        }

    private:
        std::stringstream m_log;
};

#endif