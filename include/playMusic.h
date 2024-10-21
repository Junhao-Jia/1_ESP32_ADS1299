#ifndef __PLAYMUSIC__
#define __PLAYMUSIC__

#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"
#include "Definition.h"

class playMusic : public DFRobotDFPlayerMini
{
    public:
    void playMusicInit(void);
    void playMusicTest(void);
};

#endif