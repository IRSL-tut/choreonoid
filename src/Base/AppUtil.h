#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include <QKeyEvent>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AppUtil
{
public:
    static bool isAppInitializing();
    static SignalProxy<void()> sigAppExecutionStarted();
    static SignalProxy<void()> sigAboutToQuit();
    static void updateGui(bool allEvents = false);
    static bool isNoWindowMode();
    static bool isWindowSystemAvailable();
    static bool isOffscreenMode();
    static bool isTestMode();
    static void checkErrorAndExitIfTestMode();
    static bool isNestedEventLoopActive();
    static void beginNestedEventLoop();
    static void endNestedEventLoop();
    static SignalProxy<void()> sigNestedEventLoopExited();
    static SignalProxy<void(QKeyEvent* event)> sigKeyPressed();
    static SignalProxy<void(QKeyEvent* event)> sigKeyReleased();
};

[[deprecated("Use AppUtil::sigAboutToQuit")]]
CNOID_EXPORT SignalProxy<void()> sigAboutToQuit();
[[deprecated("Use AppUtil::updateGui")]]
CNOID_EXPORT void updateGui();

}

#endif
