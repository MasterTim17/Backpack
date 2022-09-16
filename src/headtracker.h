#pragma once

#include "msp.h"
#include "msptypes.h"
#include "module_base.h"
#include <Arduino.h>

class Headtracker : public ModuleBase
{
public:
    Headtracker(Stream *port);
    void Init();
    void Loop(uint32_t now);
    void SendIndexCmd(uint8_t index){};
    void SetRecordingState(uint8_t recordingState, uint16_t delay){};
    void getHeadtrackerData(uint8_t *data); //two bytes for two axis headtracking

private:
    Stream *m_port;
    uint32_t m_last;
};