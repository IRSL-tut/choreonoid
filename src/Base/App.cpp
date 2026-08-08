#include "App.h"
#include "AppUtil.h"
#include "AppConfig.h"
#include "AppCustomizationUtil.h"
#include "ExtensionManager.h"
#include "PluginManager.h"
#include "Plugin.h"
#include "OptionManager.h"
#include "ItemManager.h"
#include "MessageView.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "ProjectBackupManager.h"
#include "UnifiedEditHistory.h"
#include "UnifiedEditHistoryView.h"
#include "ItemEditRecordManager.h"
#include "TimeSyncItemEngine.h"
#include "MainWindow.h"
#include "LayoutSwitcher.h"
#include "FolderItem.h"
#include "SubProjectItem.h"
#include "ExtCommandItem.h"
#include "SceneItem.h"
#include "ImageItem.h"
#include "SceneGeometryMeasurementTracker.h"
#include "RawSceneItem.h"
#include "CameraItem.h"
#include "LightingItem.h"
#include "PointSetItem.h"
#include "PointSetGeometryMeasurementTracker.h"
#include "MultiPointSetItem.h"
#include "AbstractTextItem.h"
#include "ScriptItem.h"
#include "MessageLogItem.h"
#include "AbstractSeqItem.h"
#include "MultiValueSeqItem.h"
#include "MultiSE3SeqItem.h"
#include "MultiSE3MatrixSeqItem.h"
#include "Vector3SeqItem.h"
#include "MultiVector3SeqItem.h"
#include "ReferencedObjectSeqItem.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "PositionTagGroupItem.h"
#include "DistanceMeasurementItem.h"
#include "ViewManager.h"
#include "ItemTreeView.h"
#include "ItemPropertyView.h"
#include "SceneView.h"
#include "LocationView.h"
#include "FileBar.h"
#include "ScriptBar.h"
#include "TimeBar.h"
#include "DisplayValueFormatBar.h"
#include "SceneBar.h"
#include "CaptureBar.h"
#include "ImageView.h"
#include "TaskView.h"
#include "GraphBar.h"
#include "MultiValueSeqGraphView.h"
#include "MultiSE3SeqGraphView.h"
#include "CoordinateFrameListView.h"
#include "TextEditView.h"
#include "GeneralSliderView.h"
#include "VirtualJoystickView.h"
#include "MainMenu.h"
#include "Licenses.h"
#include "MovieRecorderBar.h"
#include "LazyCaller.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/ConnectionSet>
#include <cnoid/MessageOut>
#include <cnoid/Config>
#include <cnoid/ValueTree>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/Format>
#include <Eigen/Core>
#include <QApplication>
#include <QTranslator>
#include <QSurfaceFormat>
#include <QStyleFactory>
#include <QThread>
#include <QLibraryInfo>
#include <QLibrary>
#include <regex>
#include <iostream>
#include <algorithm>
#include <csignal>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#include <QTextCodec>
#endif

#ifdef Q_OS_WIN32
#include <windows.h>
#include <mbctype.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

App* instance_ = nullptr;

Signal<void()> sigExecutionStarted_;
Signal<void()> sigAboutToQuit_;

bool isDoingInitialization_ = true;
int nestedEventLoopCounter = 0;
Signal<void()> sigNestedEventLoopExited_;
bool isNonInteractiveMode = false;
bool isBatchMode = false;
bool isHeadlessMode = false;
bool isWindowSystemAvailable = true;
bool isOffscreenMode = false;
bool isHeadlessModeAutoEnabled = false;
bool ctrl_c_pressed = false;
bool exitRequested = false;
bool isStartupProcessingFinished = false;
bool isQuitOptionSpecified = false;
bool isTestModeOptionSpecified = false;
vector<string> additionalPathVariables;
vector<string> pluginDirsAsPrefix;

class OngoingProcessImpl : public AppUtil::OngoingProcess
{
public:
    string name_;
    bool isFinished;

    OngoingProcessImpl(std::string_view name) : name_(name), isFinished(false) { }
    virtual ~OngoingProcessImpl() { finish(); }
    virtual const std::string& name() const override { return name_; }
    virtual void finish() override;
};

vector<OngoingProcessImpl*> ongoingProcesses;

void checkBatchModeExitCondition()
{
    if(isBatchMode && isStartupProcessingFinished && ongoingProcesses.empty()){
        /*
          The check is deferred so that a process finished immediately before another
          one begins does not terminate the application. The condition is checked again
          because it may have been changed after this function was called.
        */
        callLater(
            [](){
                if(isBatchMode && isStartupProcessingFinished && ongoingProcesses.empty()){
                    App::exit();
                }
            });
    }
}

void OngoingProcessImpl::finish()
{
    if(!isFinished){
        isFinished = true;
        auto p = std::find(ongoingProcesses.begin(), ongoingProcesses.end(), this);
        if(p != ongoingProcesses.end()){
            ongoingProcesses.erase(p);
        }
        checkBatchModeExitCondition();
    }
}

void onCtrl_C_Input(int)
{
    callLater(
        [](){
            ctrl_c_pressed = true;
            if(isBatchMode && !ongoingProcesses.empty()){
                // Report what the batch mode was waiting for
                string names;
                for(auto& process : ongoingProcesses){
                    if(!names.empty()){
                        names += ", ";
                    }
                    names += process->name_;
                }
                MessageOut::master()->putln(
                    formatR(_("The batch mode was waiting for the following processes: {0}"), names));
            }
            if(isHeadlessMode){
                App::exit();
            } else {
                MainWindow::instance()->close();
            }
        });
}

#ifdef Q_OS_WIN32
BOOL WINAPI consoleCtrlHandler(DWORD ctrlChar)
{
    callLater([](){ MainWindow::instance()->close(); });
    return FALSE;
}
#endif

}

namespace cnoid {

class App::Impl : public QObject
{
public:
    enum ShutdownState {
        ShutdownNotStarted,
        ShutdownScheduled,
        ShutdownInProgress,
        ShutdownCompleted
    };

    App* self;
    QApplication* qapplication;
    int& argc;
    char** argv;
    string appName;
    string organization;
    string pluginPathList;
    string iconFilename;
    string builtinProjectFile;
    MessageOut* mout;
    OptionManager* optionManager;
    PluginManager* pluginManager;
    ExtensionManager* ext;
    MainWindow* mainWindow;
    MessageView* messageView;

    // Set when the embedded Python interpreter (managed by the optional
    // CnoidPythonInterpreter library) has been initialized. It is called as the
    // very last step of the shutdown to finalize the interpreter safely under
    // the nanobind backend.
    void (*finalizePythonInterpreter)() = nullptr;
    QTranslator translator;
    ErrorCode error;
    string errorMessage;
    int returnCode;
    bool isAppInitialized;
    bool doQuit;
    bool doListQtStyles;
    ShutdownState shutdownState;
    bool isEventLoopRunning;
    bool isAboutToQuitSignalEmitted;
    bool doCloseMainWindowAfterShutdown;
    bool doStoreWindowStateOnShutdown;

    ScopedConnectionSet ongoingProcessConnections;
    AppUtil::OngoingProcessHandle continuousUpdateProcess;
    AppUtil::OngoingProcessHandle playbackProcess;

    Impl(App* self, int& argc, char** argv, const std::string& appName, const std::string& organization);
    ~Impl();
    void initialize();
    int exec();
    void initializeOngoingProcessDetection();
    void requestShutdown(bool doCloseMainWindow, bool doStoreWindowState);
    void performShutdown(bool areGuiUpdatesAvailable);
    void closeTopLevelWidgetsExceptMainWindow();
    void enableMessageViewRedirectToStdOut();
    virtual bool eventFilter(QObject* watched, QEvent* event);
};

}


App::App(int& argc, char** argv, const std::string& appName, const std::string& organization)
{
    impl = new Impl(this, argc, argv, appName, organization);
}


App::Impl::Impl(App* self, int& argc, char** argv, const std::string& appName, const std::string& organization)
    : self(self),
      qapplication(nullptr),
      argc(argc),
      argv(argv),
      appName(appName),
      organization(organization)
{
    instance_ = self;
    isDoingInitialization_ = true;

    mout = MessageOut::master();
    mout->setPendingMode(true);
    
    AppConfig::initialize(appName, organization);

    pluginManager = PluginManager::instance();
    if(auto pluginPathList = getenv("CNOID_PLUGIN_PATH")){
        pluginManager->addPluginPathList(toUTF8(pluginPathList));
    }

    ext = nullptr;
    mainWindow = nullptr;
    messageView = nullptr;
    error = NoError;
    returnCode = 0;
    isAppInitialized = false;
    doQuit = false;
    doListQtStyles = false;
    shutdownState = ShutdownNotStarted;
    isEventLoopRunning = false;
    isAboutToQuitSignalEmitted = false;
    doCloseMainWindowAfterShutdown = false;
    doStoreWindowStateOnShutdown = false;

    // OpenGL settings
    QSurfaceFormat glFormat = QSurfaceFormat::defaultFormat();

    // Request OpenGL 4.6 (Qt will fallback to the best available version)
    glFormat.setVersion(4, 6);
    glFormat.setProfile(QSurfaceFormat::CoreProfile);
    glFormat.setRenderableType(QSurfaceFormat::OpenGL);

    char* CNOID_DISABLE_REVERSED_DEPTH_BUFFER = getenv("CNOID_DISABLE_REVERSED_DEPTH_BUFFER");
    if(CNOID_DISABLE_REVERSED_DEPTH_BUFFER && strcmp(CNOID_DISABLE_REVERSED_DEPTH_BUFFER, "1") == 0){
        GLSceneRenderer::forceStandardDepthBuffer();
    }

    glFormat.setSwapInterval(
        AppConfig::archive()->openMapping("OpenGL")->get("vsync", false));

    QSurfaceFormat::setDefaultFormat(glFormat);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

    /*
      This attribute is necessary to render the scene on a scene view when the view is
      separated from the main window. Note that the default surface format must be
      initialized before creating the QApplication instance.
    */
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

#if (QT_VERSION >= QT_VERSION_CHECK(5, 10, 0)) && (QT_VERSION < QT_VERSION_CHECK(6, 0, 0))
    QCoreApplication::setAttribute(Qt::AA_DisableWindowContextHelpButton);
#endif

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    /*
      Prevent Qt6 from creating native windows for all sibling widgets when one widget
      requires WA_NativeWindow (e.g., GSMediaView for GStreamer video overlay).
      This fixes QRhi OpenGL context errors with native X11 rendering.
    */
    QCoreApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
#endif

    // Decide the headless/offscreen execution mode based on the available
    // window system and the --headless option.
    //
    // - If neither DISPLAY nor WAYLAND_DISPLAY is set, no window system is
    //   available, so the application is forced into headless mode and the
    //   Qt offscreen platform plugin is used. In that case the vision
    //   simulator switches its OpenGL backend to EGL (see
    //   GLVisionSensorRenderingScreen::initializeGL).
    // - If --headless is given on a system with a window system, the GUI is
    //   simply not shown but Qt keeps using the regular platform (xcb /
    //   wayland) and the vision simulator keeps using the Qt OpenGL (GLX)
    //   backend. This avoids the "QOpenGLWidget is not supported on this
    //   platform" warning that would otherwise be emitted by the offscreen
    //   platform plugin when SceneWidget (a QOpenGLWidget) is constructed.
    //
    // Note: this auto-detection is only meaningful on Unix-like systems.
    // Windows does not support OpenGL context creation with the offscreen
    // platform plugin.
#ifdef Q_OS_UNIX
    isWindowSystemAvailable =
        (getenv("DISPLAY") != nullptr) || (getenv("WAYLAND_DISPLAY") != nullptr);
    bool noWindowRequested = false;
    for(int i = 1; i < argc; ++i){
        // Note that --no-window is the old name of the --headless option
        if(strcmp(argv[i], "--headless") == 0 || strcmp(argv[i], "--no-window") == 0){
            noWindowRequested = true;
            break;
        }
    }
    if(!isWindowSystemAvailable && !noWindowRequested){
        isHeadlessModeAutoEnabled = true;
    }
    isHeadlessMode = noWindowRequested || !isWindowSystemAvailable;
    isOffscreenMode = isHeadlessMode && !isWindowSystemAvailable;
    if(isOffscreenMode){
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    /*
      Note that the message on this mode is not output here but in the initialize
      function, because the text domain of this module has not been bound yet at
      this point and the message would not be translated.
    */
#endif

    qapplication = new QApplication(argc, argv);
    qapplication->setApplicationName(appName.c_str());
    qapplication->setOrganizationName(organization.c_str());

#ifdef Q_OS_UNIX
    // See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
    setlocale(LC_NUMERIC, "C");
#endif

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
#endif

#ifdef Q_OS_WIN32
    // Make a bundled Python available if it exists in the Choreonoid top directory.
    std::smatch match;
    for(auto& dir : std::filesystem::directory_iterator(executableTopDirPath())){
        static std::regex re("^Python\\d+$");
        string dirString = dir.path().filename().string();
        if(regex_match(dirString, match, re)){
            auto pathenv = QString::fromLocal8Bit(qgetenv("PATH"));
            qputenv("PATH", QString("%1;%2").arg(dir.path().string().c_str()).arg(pathenv).toLocal8Bit());
            break;
        }
    }
#endif
}


bool App::requirePluginToCustomizeApplication(const std::string& pluginName)
{
    impl->pluginManager->loadPlugins(false);
    
    auto plugin = impl->pluginManager->findPlugin(pluginName);
    if(!plugin){
        impl->error = PluginNotFound;
        impl->errorMessage = impl->pluginManager->getErrorMessage(pluginName);
        return false;
    }

    AppCustomizationUtil util(this, impl->argc, impl->argv);
    if(!plugin->customizeApplication(util)){
        impl->error = CustomizationFailed;
        return false;
    }

    return true;
}


void App::setIcon(const std::string& filename)
{
    impl->iconFilename = filename;
    if(impl->qapplication){
        impl->qapplication->setWindowIcon(QIcon(filename.c_str()));
    }
}


void App::addPluginPath(const std::string& path)
{
    if(!path.empty()){
        impl->pluginManager->addPluginPathList(path);
    }
}


void App::setBuiltinProject(const std::string& projectFile)
{
    impl->builtinProjectFile = projectFile;
}


void App::initialize()
{
    impl->initialize();
}


void App::Impl::initialize()
{
    if(checkCurrentLocaleLanguageSupport()){

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        QString translationsPath = QLibraryInfo::path(QLibraryInfo::TranslationsPath);
#else
        QString translationsPath = QLibraryInfo::location(QLibraryInfo::TranslationsPath);
#endif
        if(translator.load("qt_" + QLocale::system().name(), translationsPath)){
            qapplication->installTranslator(&translator);
        }
    }

    qapplication->setWindowIcon(
        QIcon(!iconFilename.empty() ? iconFilename.c_str() : ":/Base/icon/choreonoid.svg"));

    auto fpvp = FilePathVariableProcessor::systemInstance();
    fpvp->restoreUserVariables(AppConfig::archive()->findMapping({ "path_variables", "pathVariables" }));
    FilePathVariableProcessor::setCurrentInstance(fpvp);

    // The text domain of this module is bound in the ExtensionManager constructor
    ext = new ExtensionManager("Base", false);

    setUTF8ToModuleTextDomain("Util");

    // The message output is deferred to here so that the message can be translated.
    // Note that the message is not lost because MessageOut is in the pending mode
    // until the message view is created.
    if(isHeadlessModeAutoEnabled){
        mout->putln(
            _("No window system is available. Choreonoid has been switched to headless mode."));
    }

    optionManager = new OptionManager(appName);

    optionManager->add_flag(
        "--batch", isBatchMode,
        "run in the non-interactive mode and exit when all the automatic processing has finished");

    optionManager->add_flag(
        "--non-interactive", isNonInteractiveMode,
        "do not expect any user interaction: put MessageView text to the standard output "
        "and do not show any dialog");

    /*
      The following options have been removed. They are still registered as hidden
      options so that a clear message can be given instead of the generic parse error
      of the option parser.
    */
    optionManager->add_flag("--quit", isQuitOptionSpecified)->group("");
    optionManager->add_flag("--test-mode", isTestModeOptionSpecified)->group("");

    // The --headless option is registered here only so that it appears in
    // the help message and is accepted as a valid command-line option by the
    // option parser. The actual value of isHeadlessMode has already been
    // determined before QApplication construction (see App::Impl::Impl),
    // because the Qt platform plugin (offscreen vs. xcb / wayland) must be
    // selected at that point, which is earlier than when OptionManager
    // parses the command line.
    //
    // To prevent OptionManager from overwriting the value of isHeadlessMode
    // that has already been determined, a dummy variable is bound here
    // instead of isHeadlessMode itself. This also avoids the situation where
    // isHeadlessMode was set to true automatically due to the absence of a
    // window system but the option parser would otherwise leave it
    // unchanged.
    //
    // The --no-window option is the old name of --headless. It is still
    // accepted but is hidden from the help message.
    static bool headlessOptionDummy = false;
    optionManager->add_flag(
        "--headless", headlessOptionDummy,
        "do not show the application window. This includes --non-interactive");
    static bool noWindowOptionDummy = false;
    optionManager->add_flag("--no-window", noWindowOptionDummy)->group("");

    optionManager->add_flag(
        "--list-qt-styles", doListQtStyles,
        "list all the available qt styles");

    optionManager->add_option(
        "--path-variable", additionalPathVariables,
        "Set a path variable in the format \"name=value\"");

    optionManager->add_option(
        "--add-plugin-dir-as-prefix", pluginDirsAsPrefix,
        "Add a plugin directory as an install path prefix");
    
    mainWindow = MainWindow::initialize(appName, ext);

    ViewManager::initializeClass(ext);

    MessageView::initializeClass(ext);
    messageView = MessageView::instance();
    mout->flushPendingMessages();
    mout->setPendingMode(false);
    
    ItemManager::initializeClass(ext);
    ProjectManager::initializeClass(ext);
    LayoutSwitcher::initializeClass(ext);
    ProjectBackupManager::initializeClass();
    RootItem::initializeClass(ext);
    UnifiedEditHistory::initializeClass(ext);
    UnifiedEditHistoryView::initializeClass(ext);
    ItemEditRecordManager::initializeClass(ext);

    /**
       Since the main menu may be customized by the main function of a custom application executable
       and the custom main menu may depend on plugins, the main menu setup is processed after
       initializing the plugin manager so that the custom main menu setup can use the functions of it.
    */
    MainMenu::instance()->setMenuItems();

    FileBar::initialize(ext);
    ScriptBar::initialize(ext);
    TimeBar::initialize(ext);
    DisplayValueFormatBar::initialize(ext);
    ItemTreeView::initializeClass(ext);
    ItemPropertyView::initializeClass(ext);
    SceneView::initializeClass(ext);
    // SceneBar must be initialized after the initialization of SceneView
    SceneBar::initialize(ext);
    LocationView::initializeClass(ext);
    ImageViewBar::initialize(ext);
    ImageView::initializeClass(ext);
    TextEditView::initializeClass(ext);
    GeneralSliderView::initializeClass(ext);
    GraphBar::initialize(ext);
    MultiValueSeqGraphView::initializeClass(ext);
    MultiSE3SeqGraphView::initializeClass(ext);
    CoordinateFrameListView::initializeClass(ext);
    TaskView::initializeClass(ext);
    VirtualJoystickView::initializeClass(ext);

    TimeSyncItemEngineManager::initializeClass(ext);
    
    FolderItem::initializeClass(ext);
    SubProjectItem::initializeClass(ext);
    ExtCommandItem::initializeClass(ext);
    AbstractSeqItem::initializeClass(ext);
    AbstractMultiSeqItem::initializeClass(ext);
    MultiValueSeqItem::initializeClass(ext);
    MultiSE3SeqItem::initializeClass(ext);
    MultiSE3MatrixSeqItem::initializeClass(ext);
    Vector3SeqItem::initializeClass(ext);
    MultiVector3SeqItem::initializeClass(ext);
    ReferencedObjectSeqItem::initializeClass(ext);
    SceneItem::initializeClass(ext);
    ImageItem::initializeClass(ext);
    SceneGeometryMeasurementTracker::initializeClass();
    RawSceneItem::initializeClass(ext);
    CameraItem::initializeClass(ext);
    LightingItem::initializeClass(ext);
    PointSetItem::initializeClass(ext);
    PointSetGeometryMeasurementTracker::initializeClass();
    MultiPointSetItem::initializeClass(ext);
    AbstractTextItem::initializeClass(ext);
    ScriptItem::initializeClass(ext);
    MessageLogItem::initializeClass(ext);
    CoordinateFrameListItem::initializeClass(ext);
    CoordinateFrameItem::initializeClass(ext);
    PositionTagGroupItem::initializeClass(ext);
    DistanceMeasurementItem::initializeClass(ext);

    MovieRecorderBar::initializeClass(ext);
    CaptureBar::initialize(ext);
    
    messageView->putln(
        formatR(_("The Eigen library version {0}.{1}.{2} is used (SIMD intruction sets in use: {3})."),
                EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION,
                Eigen::SimdInstructionSetsInUse()));

    // Optionally load the CnoidPythonInterpreter library, which manages the
    // lifecycle of the embedded Python interpreter for the nanobind backend, and
    // initialize the interpreter before loading the plugins so that the Python
    // plugin and the binding modules can rely on a running interpreter. The
    // library is loaded dynamically so that the Base module has no compile-time
    // dependency on Python; when it is absent (Python disabled or the pybind11
    // backend), this simply does nothing. The QLibrary object can be deleted
    // afterwards because Qt does not unload the library when it is destroyed.
    {
        auto lib = new QLibrary("CnoidPythonInterpreter");
        if(lib->load()){
            auto initFunc =
                reinterpret_cast<int(*)()>(lib->resolve("cnoid_initializePythonInterpreter"));
            auto finalizeFunc =
                reinterpret_cast<void(*)()>(lib->resolve("cnoid_finalizePythonInterpreter"));
            if(initFunc && finalizeFunc && initFunc()){
                finalizePythonInterpreter = finalizeFunc;
            }
        }
        delete lib;
    }

    pluginManager->doStartupLoading();

    mainWindow->installEventFilter(this);

    /*
      Some plugins such as OpenRTM plugin are driven by a library which tries to catch SIGINT.
      This may block the normal termination by inputting Ctrl+C.
      To avoid it, the following signal handliers are set.
    */
    std::signal(SIGINT, onCtrl_C_Input);
    std::signal(SIGTERM, onCtrl_C_Input);

#ifdef Q_OS_WIN32
    /*
      The above SIGINT handler seems to work even on Windows
      when Choreonoid is compiled as a console-program,
      and the following handler only works for a console-program, too.
      Hence the folloing handler for Windows is currently disabled.
    */
    // SetConsoleCtrlHandler(consoleCtrlHandler, TRUE);
#endif

    if(builtinProjectFile.empty()){
        builtinProjectFile = ":/Base/project/layout.cnoid";
    }
    ProjectManager::instance()->loadBuiltinProject(builtinProjectFile);

    isAppInitialized = true;
}


App::~App()
{
    if(impl){
        delete impl;
    }
}


App::Impl::~Impl()
{
    delete qapplication;
    delete pluginManager;
    AppConfig::flush();
}


int App::exec()
{
    return impl->exec();
}


int App::Impl::exec()
{
    if(!isAppInitialized){
        initialize();
    }
        
    isDoingInitialization_ = false;

    try{
#ifdef Q_OS_WIN32
        argv = optionManager->ensure_utf8(argv);
#endif
        optionManager->parse(argc, argv);

        if(isQuitOptionSpecified || isTestModeOptionSpecified){
            /*
              The message is put to the standard error output because the removed
              options are command line errors and the message view is not available
              when the application does not run in the non-interactive mode.
            */
            auto putRemovedOptionMessage =
                [](const char* option){
                    cerr << fromUTF8(
                        formatR(_("The {0} option has been removed. Use the --batch option instead."),
                                option))
                         << endl;
                };
            if(isQuitOptionSpecified){
                putRemovedOptionMessage("--quit");
            }
            if(isTestModeOptionSpecified){
                putRemovedOptionMessage("--test-mode");
            }
            returnCode = 1;
            doQuit = true;
        }
        if(isBatchMode){
            isNonInteractiveMode = true;
        }
        if(isHeadlessMode){
            isNonInteractiveMode = true;
        }
        if(isNonInteractiveMode){
            enableMessageViewRedirectToStdOut();
        }
        if(!additionalPathVariables.empty()){
            auto fpvp = FilePathVariableProcessor::systemInstance();
            std::regex re("^([a-zA-Z][a-zA-Z_0-9]*)=([^;-?[\\]^'{-~]+)$");
            std::smatch match;
            for(auto& var : additionalPathVariables){
                if(regex_match(var, match, re)){
                    string name = match.str(1);
                    string path = match.str(2);
                    fpvp->addUserVariable(name, path);
                }
            }
        }
        if(!doQuit && doListQtStyles){
            cout << QStyleFactory::keys().join(" ").toStdString() << endl;
            doQuit = true;
        }

        for(auto& prefix : pluginDirsAsPrefix){
            pluginManager->addPluginDirectoryAsPrefix(prefix);
        }
        
        optionManager->processOptionsPhase1();

    } catch (const CLI::ParseError& error) {
        optionManager->exit(error);
        doQuit = true;
    }

    if(!doQuit){
        if(isBatchMode){
            initializeOngoingProcessDetection();
        }
        if(mainWindow->isVisible()){
            AppUtil::updateGui();
        } else {
            if(!isHeadlessMode){
                mainWindow->show();
                mainWindow->waitForWindowSystemToActivate();
            }
        }
        callLater(
            [this](){
                auto process = AppUtil::beginOngoingProcess(_("startup processing"));
                optionManager->processOptionsPhase2();
                sigExecutionStarted_();
                isStartupProcessingFinished = true;
                process->finish();
            });

        isEventLoopRunning = true;
        int result = qapplication->exec();
        isEventLoopRunning = false;

        if(result != 0){
            returnCode = result;
        }
    }

    if(shutdownState != ShutdownCompleted){
        // This is a fallback for the headless mode, for the case where the event
        // loop is not executed at all, and for an event-loop exit that does not
        // originate from the main-window close sequence. No widget update is
        // allowed here because a native window may already have been destroyed.
        performShutdown(false);
    }

    if(returnCode == 0 && messageView->hasErrorMessages()){
        returnCode = 1;
    }

    delete mainWindow;
    mainWindow = nullptr;
    messageView = nullptr;
    MessageView::unblockFlush();

    /*
      The singleton instances of MessageOut are released here because they are owned
      by their Python wrapper objects once they have been exposed to Python, and the
      references kept by the MessageOut class would otherwise make the wrappers
      outlive the interpreter. Note that MessageOut must not be used after this.
    */
    MessageOut::releaseSingletons();

    /*
      Finalize the embedded Python interpreter as the very last step of the shutdown.
      Under the nanobind backend this is when the Python wrappers that still own
      Referenced-derived C++ objects are destroyed, so it must run after all the
      plugins have been finalized and the item tree has been released. It must also
      run after the widgets have been destroyed, because a widget can hold a Python
      callback whose release requires a valid interpreter.
    */
    if(finalizePythonInterpreter){
        finalizePythonInterpreter();
        finalizePythonInterpreter = nullptr;
    }

    // Note that the application must be terminated without deleting
    // the base extension manager pointed by the 'ext' variable
    // to avoid crashes due to destructors accessing invalid objects.
    
    return returnCode;
}


App::ErrorCode App::error() const
{
    return impl->error;
}


const std::string& App::errorMessage() const
{
    return impl->errorMessage;
}


bool App::isDoingInitialization()
{
    return isDoingInitialization_;
}


ExtensionManager* App::baseModule()
{
    return instance_->impl->ext;
}


bool App::Impl::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == mainWindow && event->type() == QEvent::Close){
        if(shutdownState == ShutdownCompleted){
            event->accept();
            return true;
        }
        if(shutdownState != ShutdownNotStarted){
            event->ignore();
            return true;
        }
        bool doShutdown;
        if(ctrl_c_pressed || exitRequested){
            doShutdown = true;
            /*
              The confirmation dialog cannot be used in this case, but the directory
              extracted from a project pack should still be removed if it does not
              have any modifications.
            */
            ProjectManager::instance()->confirmToRemoveUnpackedProjectPack(false);
        } else {
            doShutdown = ProjectManager::instance()->tryToCloseProject();
            if(doShutdown){
                ProjectManager::instance()->confirmToRemoveUnpackedProjectPack();
            }
        }
        if(doShutdown){
            // Keep the native window alive until all teardown operations that
            // can update views have been completed.
            event->ignore();
            requestShutdown(true, true);
        } else {
            event->ignore();
        }
        return true;
    }
    return false;
}


/**
   This function sets up the detection of the processes that must keep the application
   running in the batch mode. Note that an item in the continuous update state is handled
   as an ongoing process here, and it covers most of the automatic processing including
   the simulation, so that each class does not have to register its own process.
*/
void App::Impl::initializeOngoingProcessDetection()
{
    ongoingProcessConnections.add(
        RootItem::instance()->sigTreeContinuousUpdateStateExistenceChanged().connect(
            [this](bool on){
                if(on){
                    if(!continuousUpdateProcess){
                        continuousUpdateProcess =
                            AppUtil::beginOngoingProcess(_("continuous update of items"));
                    }
                } else {
                    continuousUpdateProcess.reset();
                }
            }));

    auto timeBar = TimeBar::instance();

    ongoingProcessConnections.add(
        timeBar->sigPlaybackStarted().connect(
            [this](double){
                if(!playbackProcess){
                    playbackProcess = AppUtil::beginOngoingProcess(_("playback"));
                }
            }));

    ongoingProcessConnections.add(
        timeBar->sigPlaybackStopped().connect(
            [this](double, bool){ playbackProcess.reset(); }));
}


void App::Impl::requestShutdown(bool doCloseMainWindow, bool doStoreWindowState)
{
    doCloseMainWindowAfterShutdown |= doCloseMainWindow;
    doStoreWindowStateOnShutdown |= doStoreWindowState;

    if(shutdownState == ShutdownNotStarted){
        shutdownState = ShutdownScheduled;
        callLater(
            [this](){
                performShutdown(
                    doCloseMainWindowAfterShutdown && mainWindow && mainWindow->isVisible());
            });
    }
}


void App::Impl::performShutdown(bool areGuiUpdatesAvailable)
{
    if(shutdownState == ShutdownInProgress || shutdownState == ShutdownCompleted){
        return;
    }

    /*
      The shutdown must not be performed from a nested event loop such as the one
      processed by AppUtil::updateGui. A typical case is a Python script that calls
      App::exit and then outputs a message, because flushing the message processes
      the pending Qt events and the shutdown would destroy the plugins and finalize
      the Python interpreter while the script frame is still alive on the stack.
      In that case the shutdown is put back into the event queue so that it is
      performed after the stack has returned to the main event loop.
    */
    if(AppUtil::isNestedEventLoopActive()){
        callLater(
            [this, areGuiUpdatesAvailable](){ performShutdown(areGuiUpdatesAvailable); });
        return;
    }

    shutdownState = ShutdownInProgress;

    // Prevent message flushing from processing pending Qt events while plugin
    // and item objects are being destroyed. Direct MessageView updates remain
    // enabled until the teardown has finished if the native window is valid.
    MessageView::blockFlush();
    if(!areGuiUpdatesAvailable && messageView){
        messageView->setGuiUpdatesEnabled(false);
    }

    if(!isAboutToQuitSignalEmitted){
        isAboutToQuitSignalEmitted = true;
        sigAboutToQuit_();
    }

    if(doStoreWindowStateOnShutdown && mainWindow){
        mainWindow->storeWindowStateConfig();
    }

    if(mainWindow){
        /*
          Disable the repainting of the widgets before the teardown begins. The
          following operations destroy widgets including the QOpenGLWidget-based
          views, and repainting the window while it is being done can access an
          already destroyed platform window. Note that the native window itself
          is kept alive here so that the direct MessageView updates remain safe.
        */
        mainWindow->setUpdatesEnabled(false);
    }

    UnifiedEditHistory::instance()->terminateRecording();
    RootItem::instance()->clearChildren();

    // The removal must be done after the item tree is cleared so that the files
    // in the extracted directory are not locked by the items using them
    ProjectManager::instance()->removeUnpackedProjectPackIfDecided();

    /*
      The views must be deleted here because a view can keep references to items
      and other objects whose wrapper objects are owned by the Python interpreter.
      The views would otherwise be deleted together with the main window, which is
      deleted after the interpreter has been finalized. Note that this must be done
      before the plugins and the managed objects are finalized because deactivating
      a view can access a tool bar managed by them. The message view is kept alive
      because the remaining shutdown code still uses it.
    */
    for(auto& view : ViewManager::allViews()){
        if(view != messageView){
            ViewManager::deleteView(view);
        }
    }

    pluginManager->finalizePlugins();
    ext->deleteManagedObjects();

    // The item class registry keeps the singleton item instances such as RootItem.
    // They must be released here because the extension manager of the base module
    // is not deleted on shutdown and the instances would otherwise outlive the
    // Python interpreter that owns their wrapper objects.
    ItemManager::finalizeClass();

    // Any message arriving after this point may still be forwarded to a
    // non-GUI sink, but it must not touch widgets while the windows are closed.
    if(messageView){
        messageView->setGuiUpdatesEnabled(false);
    }

    shutdownState = ShutdownCompleted;

    if(doCloseMainWindowAfterShutdown && mainWindow){
        closeTopLevelWidgetsExceptMainWindow();
        mainWindow->close();
    }

    if(isEventLoopRunning){
        qapplication->exit(returnCode);
    }
}


void App::Impl::closeTopLevelWidgetsExceptMainWindow()
{
    QWidgetList windows = QApplication::topLevelWidgets();
    for(int i=0; i < windows.size(); ++i){
        QWidget* window = windows[i];
        if(window != mainWindow){
            window->close();
        }
    }
}


void App::exit(int returnCode)
{
    if(instance_){
        auto impl = instance_->impl;
        impl->returnCode = returnCode;
        exitRequested = true;
        if(isHeadlessMode){
            impl->requestShutdown(false, false);
        } else if(impl->mainWindow){
            impl->mainWindow->close();
        }
    }
}


void App::Impl::enableMessageViewRedirectToStdOut()
{
    static bool isInitialized = false;

    if(!isInitialized){
        auto mv = instance_->impl->messageView;
        cout << fromUTF8(mv->messages());
        cout.flush();
        mv->sigMessage().connect(
            [this](const std::string& text){
                std::cout << fromUTF8(text);
                std::cout.flush();
            });
        isInitialized = true;
    }
}


SignalProxy<void()> App::sigExecutionStarted()
{
    return sigExecutionStarted_;
}


bool AppUtil::isAppInitializing()
{
    return isDoingInitialization_;
}


bool AppUtil::isAppShuttingDown()
{
    if(instance_){
        return instance_->impl->shutdownState != App::Impl::ShutdownNotStarted;
    }
    return false;
}


SignalProxy<void()> AppUtil::sigAppExecutionStarted()
{
    return sigExecutionStarted_;
}


bool AppUtil::isNonInteractiveMode()
{
    return ::isNonInteractiveMode;
}


bool AppUtil::isBatchMode()
{
    return ::isBatchMode;
}


AppUtil::OngoingProcessHandle AppUtil::beginOngoingProcess(std::string_view name)
{
    auto process = new OngoingProcessImpl(name);
    ongoingProcesses.push_back(process);
    return process;
}


std::vector<std::string> AppUtil::ongoingProcessNames()
{
    std::vector<std::string> names;
    names.reserve(ongoingProcesses.size());
    for(auto& process : ongoingProcesses){
        names.push_back(process->name_);
    }
    return names;
}


SignalProxy<void(QKeyEvent* event)> AppUtil::sigKeyPressed()
{
    //! \todo Support top windows other than the main window
    return instance_->impl->mainWindow->sigKeyPressed();
}


SignalProxy<void(QKeyEvent* event)> AppUtil::sigKeyReleased()
{
    //! \todo Support top windows other than the main window
    return instance_->impl->mainWindow->sigKeyReleased();
}


SignalProxy<void()> App::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


SignalProxy<void()> AppUtil::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


SignalProxy<void()> cnoid::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


void App::updateGui(bool allEvents)
{
    AppUtil::updateGui(allEvents);
}


bool App::isNestedEventLoopActive()
{
    return nestedEventLoopCounter > 0;
}


void App::beginNestedEventLoop()
{
    ++nestedEventLoopCounter;
}


void App::endNestedEventLoop()
{
    if(--nestedEventLoopCounter == 0){
        sigNestedEventLoopExited_();
    }
}


SignalProxy<void()> App::sigNestedEventLoopExited()
{
    return sigNestedEventLoopExited_;
}


void AppUtil::updateGui(bool allEvents)
{
    ++nestedEventLoopCounter;
    if(allEvents){
        QCoreApplication::processEvents();
    } else {
        QCoreApplication::processEvents(
            QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers);
    }
    --nestedEventLoopCounter;
}


bool AppUtil::isNestedEventLoopActive()
{
    return nestedEventLoopCounter > 0;
}


void AppUtil::beginNestedEventLoop()
{
    ++nestedEventLoopCounter;
}


void AppUtil::endNestedEventLoop()
{
    if(--nestedEventLoopCounter == 0){
        sigNestedEventLoopExited_();
    }
}


SignalProxy<void()> AppUtil::sigNestedEventLoopExited()
{
    return sigNestedEventLoopExited_;
}


bool AppUtil::isHeadlessMode()
{
    return ::isHeadlessMode;
}


bool AppUtil::isWindowSystemAvailable()
{
    return ::isWindowSystemAvailable;
}


bool AppUtil::isOffscreenMode()
{
    return ::isOffscreenMode;
}


void cnoid::updateGui()
{
    return AppUtil::updateGui();
}
