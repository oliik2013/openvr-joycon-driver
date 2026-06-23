#ifndef _JSL_GLUE_H_
#define _JSL_GLUE_H_

#include <functional>
#include <JSL/JoyShockLibrary.h>

class JoyconDriver;

class JSLGlue
{
public:
    JSLGlue();

    void init();
    void cleanup();

    void enable_callback();
    void disable_callback();

    void setDriver(bool isRight, JoyconDriver *driver);

    bool isLeftConnected, isRightConnected;
    int leftHandle, rightHandle;

    JoyconDriver *leftDriver;
    JoyconDriver *rightDriver;

public:
    static JSLGlue instance;
};

#endif
