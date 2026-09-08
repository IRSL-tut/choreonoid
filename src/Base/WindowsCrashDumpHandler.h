#ifndef CNOID_BASE_WINDOWS_CRASH_DUMP_HANDLER_H
#define CNOID_BASE_WINDOWS_CRASH_DUMP_HANDLER_H

#include <string>

namespace cnoid {

/**
   This class makes the application write a crash dump file by itself when it terminates
   abnormally. The dump file can be opened with a debugger such as Visual Studio to obtain
   the call stack and the module list of the crashed process.

   The function is intended for the situation where a crash only occurs in a particular
   user environment and the developer cannot attach a debugger to it. Windows Error
   Reporting provides a similar local dump function, but it is a system-wide feature that
   must be enabled by editing the registry with the administrator privilege, which is often
   unavailable on a managed PC. This handler is built into the application itself and works
   with the privilege of a normal user, so that a user who encounters a crash only has to
   send the generated file.

   The class is only available on Windows and is compiled only there. On the other
   platforms a core dump can be obtained with the standard means of the system, so no
   counterpart of this class is provided. The class is used only by the App class inside
   the Base module and is not exported.
*/
class WindowsCrashDumpHandler
{
public:
    /**
       Install the handler. This should be called as early as possible in the application
       initialization so that a crash in the initialization process can also be captured.

       The handler is not installed unless the user requests it, because the generated files
       contain the information on the internal state of the process and the environment of
       the user. The App class installs the handler when the CNOID_CRASH_DUMP environment
       variable is set. See the constructor of App::Impl for the details.

       The files are written to the "CrashDumps" directory under the local application data
       directory of the user, which is
       "%LOCALAPPDATA%\<organization>\<application>\CrashDumps" by default. The directory
       can be changed with the CNOID_CRASH_DUMP_DIR environment variable.

       \param isFullDumpEnabled If true, the dump file contains the whole memory of the
       process. The file becomes much larger (hundreds of megabytes) but the contents of the
       heap objects can also be inspected. Otherwise a minidump is written, which contains
       the call stacks of all the threads and the module list.

       \return True if the handler has been installed.
    */
    static bool install(
        const std::string& applicationName, const std::string& organizationName,
        bool isFullDumpEnabled = false);

    /**
       The notification dialog that tells the location of the written dump file is enabled by
       default. It must be disabled for a process that runs without user interaction so that
       the process does not keep waiting for the modal dialog to be closed. The notification
       is put to the standard error output in that case.
    */
    static void setNotificationDialogEnabled(bool on);

    /**
       The message of the notification dialog is composed when the handler is installed.
       Since the handler is usually installed before the message catalogs are bound so that
       the crash in the early initialization can also be captured, this function must be
       called to apply the translation after the catalogs become available.
    */
    static void updateNotificationDialogMessage();
};

}

#endif
