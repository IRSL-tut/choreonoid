#ifndef CNOID_BASE_APP_H
#define CNOID_BASE_APP_H

#include <cnoid/Signal>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class CNOID_EXPORT App
{
        
public:
    App(int& argc, char** argv, const std::string& appName, const std::string& organization);
    ~App();

    [[deprecated("Use PluginManager::addPluginPathList")]]
    void addPluginPath(const std::string& path);

    bool requirePluginToCustomizeApplication(const std::string& pluginName);

    // Optional setting
    void setIcon(const std::string& filename);

    void setBuiltinProject(const std::string& projectFile);

    /**
       This function is only used to execute some initialization code after the basic App initialization has been finished.
       Even if this function is not explicitly called, the function is called inside the exec function.
    */
    void initialize();

    /**
       This function must be called to execute the application.
    */
    int exec();

    enum ErrorCode { NoError, PluginNotFound, CustomizationFailed };
    ErrorCode error() const;
    const std::string& errorMessage() const;

    static ExtensionManager* baseModule();
    static void processEvents();
    static void exit(int returnCode = 0);

    [[deprecated("Use AppUtil::isAppInitializing")]]
    static bool isDoingInitialization();
    [[deprecated("Use AppUtil::updateGui")]]
    static void updateGui(bool allEvents = false);
    [[deprecated("Use AppUtil::checkErrorAndExitIfTestMode")]]
    static void checkErrorAndExitIfTestMode();
    [[deprecated("Use AppUtil::sigAppExecutionStarted")]]
    static SignalProxy<void()> sigExecutionStarted();
    [[deprecated("Use AppUtil::sigAboutToQuit")]]
    static SignalProxy<void()> sigAboutToQuit();
    [[deprecated("Use AppUtil::isNestedEventLoopActive")]]
    static bool isNestedEventLoopActive();
    [[deprecated("Use AppUtil::beginNestedEventLoop")]]
    static void beginNestedEventLoop();
    [[deprecated("Use AppUtil::endNestedEventLoop")]]
    static void endNestedEventLoop();
    [[deprecated("Use AppUtil::sigNestedEventLoopExited")]]
    static SignalProxy<void()> sigNestedEventLoopExited();
        
private:
    class Impl;
    Impl* impl;

    friend class AppUtil;
};

}

#endif
