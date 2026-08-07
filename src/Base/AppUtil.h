#ifndef CNOID_BASE_APP_UTIL_H
#define CNOID_BASE_APP_UTIL_H

#include <cnoid/Signal>
#include <cnoid/Referenced>
#include <QKeyEvent>
#include <string>
#include <string_view>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT AppUtil
{
public:
    static bool isAppInitializing();
    static SignalProxy<void()> sigAppExecutionStarted();
    static SignalProxy<void()> sigAboutToQuit();
    static void updateGui(bool allEvents = false);

    /**
       The headless mode is enabled by the --headless option, and it is also enabled
       automatically when no window system is available. In this mode the application
       window is not shown. The mode also enables the non-interactive mode.
    */
    static bool isHeadlessMode();

    static bool isWindowSystemAvailable();
    static bool isOffscreenMode();

    /**
       The non-interactive mode is enabled by the --non-interactive option. In this
       mode the application does not expect any user interaction, so the MessageView
       text is put to the standard output and no dialog is shown. The mode is also
       enabled when the --headless or --batch option is specified.
    */
    static bool isNonInteractiveMode();

    /**
       The batch mode is enabled by the --batch option. In this mode the application
       exits when the startup processing has finished and no ongoing process remains.
       The mode also enables the non-interactive mode.
    */
    static bool isBatchMode();

    class OngoingProcess : public Referenced
    {
    public:
        virtual const std::string& name() const = 0;
        /**
           \note This function is called in the destructor, so resetting the handle
           can also be used to finish the process.
        */
        virtual void finish() = 0;
    };
    typedef ref_ptr<OngoingProcess> OngoingProcessHandle;

    /**
       This function registers a process that is running automatically. While at least
       one ongoing process exists, the batch mode does not exit the application. The
       process is finished when the returned handle is released.

       Note that an item in the continuous update state is automatically handled as an
       ongoing process, so this function is not necessary for a process based on it.

       \param name Description of the process. It is used to report what the batch mode
       is waiting for.
    */
    static OngoingProcessHandle beginOngoingProcess(std::string_view name);

    //! This function returns the names of the ongoing processes for diagnostic purposes.
    static std::vector<std::string> ongoingProcessNames();

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
