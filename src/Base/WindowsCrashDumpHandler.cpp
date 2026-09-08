#include "WindowsCrashDumpHandler.h"
#include <cnoid/Config>
#include <cnoid/Format>
#include <string>
#include <windows.h>
#include <dbghelp.h>
#include <csignal>
#include <cstdlib>
#include <exception>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

/*
  The functions in this file that may be executed after a crash must not use the C runtime
  library and must not allocate any memory, because the process is already in a broken
  state at that point and a function that acquires a lock of the runtime library can easily
  dead-lock there. Only the Win32 API functions that can be used in an exception filter and
  the plain buffer operations implemented below are used in them. All the resources needed
  by the handler including the message strings are prepared in advance when the handler is
  installed.
*/

constexpr int MaxNumDumpFiles = 10;
constexpr size_t PathBufferSize = 1024;
constexpr size_t MessageBufferSize = 1024;
constexpr size_t ReportBufferSize = 32768;
constexpr int MaxNumStackFrames = 64;
constexpr int MaxNumModules = 512;
constexpr int MaxSymbolNameLength = 512;

// The application-defined exception code used for a fatal error that is not an exception
constexpr DWORD FatalErrorExceptionCode = 0xe0000001;

typedef BOOL (WINAPI *MiniDumpWriteDumpFunc)(
    HANDLE hProcess, DWORD processId, HANDLE hFile, MINIDUMP_TYPE dumpType,
    PMINIDUMP_EXCEPTION_INFORMATION exceptionParam,
    PMINIDUMP_USER_STREAM_INFORMATION userStreamParam,
    PMINIDUMP_CALLBACK_INFORMATION callbackParam);

// The functions used to put the function names and the source lines into the report
typedef DWORD (WINAPI *SymSetOptionsFunc)(DWORD options);
typedef BOOL (WINAPI *SymInitializeWFunc)(HANDLE hProcess, PCWSTR searchPath, BOOL fInvadeProcess);
typedef BOOL (WINAPI *SymCleanupFunc)(HANDLE hProcess);
typedef BOOL (WINAPI *SymFromAddrWFunc)(
    HANDLE hProcess, DWORD64 address, PDWORD64 displacement, PSYMBOL_INFOW symbol);
typedef BOOL (WINAPI *SymGetLineFromAddrW64Func)(
    HANDLE hProcess, DWORD64 address, PDWORD displacement, PIMAGEHLP_LINEW64 line);
typedef BOOL (WINAPI *SymGetModuleInfoW64Func)(
    HANDLE hProcess, DWORD64 address, PIMAGEHLP_MODULEW64 moduleInfo);

// The function used to put the loaded module list into the report
typedef BOOL (WINAPI *EnumProcessModulesFunc)(
    HANDLE hProcess, HMODULE* modules, DWORD size, LPDWORD sizeNeeded);

bool installed = false;
bool notificationDialogEnabled = true;
bool isFullDumpEnabled = false;

MiniDumpWriteDumpFunc miniDumpWriteDump = nullptr;
SymSetOptionsFunc symSetOptions = nullptr;
SymInitializeWFunc symInitializeW = nullptr;
SymCleanupFunc symCleanup = nullptr;
SymFromAddrWFunc symFromAddrW = nullptr;
SymGetLineFromAddrW64Func symGetLineFromAddrW64 = nullptr;
SymGetModuleInfoW64Func symGetModuleInfoW64 = nullptr;
EnumProcessModulesFunc enumProcessModules = nullptr;
LPTOP_LEVEL_EXCEPTION_FILTER previousExceptionFilter = nullptr;
LONG crashCounter = 0;

string applicationName;

wchar_t dumpDirectoryW[PathBufferSize];
wchar_t executableFileW[PathBufferSize];
// The base name of a dump file, which is the base name of the executable file
wchar_t fileBaseNameW[128];

wchar_t notificationCaptionW[256];
// The notification message is divided into the parts before and after the dump file path
wchar_t notificationPrefixW[MessageBufferSize];
wchar_t notificationSuffixW[MessageBufferSize];

/*
  The buffers used to compose the report. They are static so that the stack of the crashed
  thread, which may be almost exhausted, is not consumed by them.
*/
wchar_t reportBuffer[ReportBufferSize];
char reportUtf8Buffer[ReportBufferSize * 3];
ULONG64 stackFrames[MaxNumStackFrames];
CONTEXT unwindContext;


/**
   A minimum text writer that does not use the C runtime library so that it can also be used
   after a crash. Note that the wsprintf function of the Win32 API is not used, either,
   because it does not support the 64-bit integer values.
*/
class TextBuffer
{
public:
    TextBuffer(wchar_t* buf, size_t capacity)
        : buf(buf), capacity(capacity), length(0) {
        buf[0] = L'\0';
    }
    const wchar_t* text() const { return buf; }
    size_t size() const { return length; }

    void put(wchar_t c){
        if(length + 1 < capacity){
            buf[length++] = c;
            buf[length] = L'\0';
        }
    }
    void put(const wchar_t* text){
        while(*text && (length + 1 < capacity)){
            buf[length++] = *text++;
        }
        buf[length] = L'\0';
    }
    void putAscii(const char* text){
        while(*text && (length + 1 < capacity)){
            buf[length++] = static_cast<wchar_t>(*text++);
        }
        buf[length] = L'\0';
    }
    void putUInt(unsigned long long value, int numMinDigits = 1){
        wchar_t digits[24];
        int n = 0;
        do {
            digits[n++] = static_cast<wchar_t>(L'0' + (value % 10));
            value /= 10;
        } while(value > 0 && n < 24);
        while(n < numMinDigits && n < 24){
            digits[n++] = L'0';
        }
        while(n > 0){
            put(digits[--n]);
        }
    }
    void putHex(unsigned long long value, int numDigits){
        put(L"0x");
        for(int i = numDigits - 1; i >= 0; --i){
            int digit = static_cast<int>((value >> (i * 4)) & 0xf);
            put(static_cast<wchar_t>(digit < 10 ? (L'0' + digit) : (L'A' + digit - 10)));
        }
    }
    void putHexValue(unsigned long long value){
        int numDigits = 1;
        for(unsigned long long v = value; v >= 16; v /= 16){
            ++numDigits;
        }
        putHex(value, numDigits);
    }
    void putPointer(const void* pointer){
        putHex(reinterpret_cast<unsigned long long>(pointer), 16);
    }
    void putPadded(const wchar_t* text, size_t width){
        size_t begin = length;
        put(text);
        while((length - begin) < width){
            put(L' ');
        }
    }
    void putDateTime(const SYSTEMTIME& time, bool doInsertSeparators){
        putUInt(time.wYear, 4);
        if(doInsertSeparators) put(L'-');
        putUInt(time.wMonth, 2);
        if(doInsertSeparators) put(L'-');
        putUInt(time.wDay, 2);
        put(doInsertSeparators ? L' ' : L'_');
        putUInt(time.wHour, 2);
        if(doInsertSeparators) put(L':');
        putUInt(time.wMinute, 2);
        if(doInsertSeparators) put(L':');
        putUInt(time.wSecond, 2);
    }

private:
    wchar_t* buf;
    size_t capacity;
    size_t length;
};


bool getEnvironmentValue(const wchar_t* name, wchar_t* buf, DWORD size)
{
    DWORD n = GetEnvironmentVariableW(name, buf, size);
    return (n > 0) && (n < size);
}


std::wstring toWideString(const std::string& utf8Text)
{
    if(utf8Text.empty()){
        return std::wstring();
    }
    int size = MultiByteToWideChar(CP_UTF8, 0, utf8Text.c_str(), -1, nullptr, 0);
    if(size <= 1){
        return std::wstring();
    }
    std::wstring wide(size - 1, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8Text.c_str(), -1, &wide[0], size);
    return wide;
}


/**
   Make a string usable as a file or directory name. Note that a name ending with a period
   or a space is also avoided because such a name cannot be handled by some of the file
   system functions even though the Win32 API silently trims those characters.
*/
std::wstring toFileNameString(const std::string& text)
{
    std::wstring name = toWideString(text);
    for(auto& c : name){
        if(c < 0x20 || c == L'<' || c == L'>' || c == L':' || c == L'"' ||
           c == L'/' || c == L'\\' || c == L'|' || c == L'?' || c == L'*'){
            c = L'_';
        }
    }
    while(!name.empty() && (name.back() == L'.' || name.back() == L' ')){
        name.pop_back();
    }
    return name;
}


bool createDirectories(wchar_t* path)
{
    for(wchar_t* p = path + 1; *p; ++p){
        if(*p == L'\\'){
            *p = L'\0';
            // The failure of an intermediate component such as a drive name is ignored
            CreateDirectoryW(path, nullptr);
            *p = L'\\';
        }
    }
    if(CreateDirectoryW(path, nullptr)){
        return true;
    }
    return GetLastError() == ERROR_ALREADY_EXISTS;
}


/**
   Remove the old dump files so that repeated crashes do not fill up the storage. This is
   executed when the handler is installed and is not executed after a crash.
*/
void removeOldDumpFiles()
{
    static const int MaxNumFiles = 64;
    struct FileInfo {
        wchar_t name[MAX_PATH];
        FILETIME time;
    };
    static FileInfo files[MaxNumFiles];
    int numFiles = 0;

    wchar_t pattern[PathBufferSize];
    TextBuffer patternText(pattern, PathBufferSize);
    patternText.put(dumpDirectoryW);
    patternText.put(L"\\*.dmp");

    WIN32_FIND_DATAW data;
    HANDLE handle = FindFirstFileW(pattern, &data);
    if(handle == INVALID_HANDLE_VALUE){
        return;
    }
    do {
        if(!(data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && numFiles < MaxNumFiles){
            lstrcpynW(files[numFiles].name, data.cFileName, MAX_PATH);
            files[numFiles].time = data.ftLastWriteTime;
            ++numFiles;
        }
    } while(FindNextFileW(handle, &data));

    FindClose(handle);

    /*
      A new dump file is added just after this function, so the oldest files are removed
      until the number of the remaining files becomes less than the maximum number.
    */
    while(numFiles >= MaxNumDumpFiles){
        int oldest = 0;
        for(int i = 1; i < numFiles; ++i){
            if(CompareFileTime(&files[i].time, &files[oldest].time) < 0){
                oldest = i;
            }
        }
        wchar_t file[PathBufferSize];
        TextBuffer fileText(file, PathBufferSize);
        fileText.put(dumpDirectoryW);
        fileText.put(L'\\');
        fileText.put(files[oldest].name);
        DeleteFileW(file);

        // Remove the report file corresponding to the dump file
        size_t n = fileText.size();
        if(n > 4){
            file[n - 3] = L't';
            file[n - 2] = L'x';
            file[n - 1] = L't';
            DeleteFileW(file);
        }
        files[oldest] = files[numFiles - 1];
        --numFiles;
    }
}


const wchar_t* getExceptionCodeName(DWORD code)
{
    switch(code){
    case EXCEPTION_ACCESS_VIOLATION:         return L"EXCEPTION_ACCESS_VIOLATION";
    case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:    return L"EXCEPTION_ARRAY_BOUNDS_EXCEEDED";
    case EXCEPTION_BREAKPOINT:               return L"EXCEPTION_BREAKPOINT";
    case EXCEPTION_DATATYPE_MISALIGNMENT:    return L"EXCEPTION_DATATYPE_MISALIGNMENT";
    case EXCEPTION_FLT_DENORMAL_OPERAND:     return L"EXCEPTION_FLT_DENORMAL_OPERAND";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO:       return L"EXCEPTION_FLT_DIVIDE_BY_ZERO";
    case EXCEPTION_FLT_INEXACT_RESULT:       return L"EXCEPTION_FLT_INEXACT_RESULT";
    case EXCEPTION_FLT_INVALID_OPERATION:    return L"EXCEPTION_FLT_INVALID_OPERATION";
    case EXCEPTION_FLT_OVERFLOW:             return L"EXCEPTION_FLT_OVERFLOW";
    case EXCEPTION_FLT_STACK_CHECK:          return L"EXCEPTION_FLT_STACK_CHECK";
    case EXCEPTION_FLT_UNDERFLOW:            return L"EXCEPTION_FLT_UNDERFLOW";
    case EXCEPTION_ILLEGAL_INSTRUCTION:      return L"EXCEPTION_ILLEGAL_INSTRUCTION";
    case EXCEPTION_IN_PAGE_ERROR:            return L"EXCEPTION_IN_PAGE_ERROR";
    case EXCEPTION_INT_DIVIDE_BY_ZERO:       return L"EXCEPTION_INT_DIVIDE_BY_ZERO";
    case EXCEPTION_INT_OVERFLOW:             return L"EXCEPTION_INT_OVERFLOW";
    case EXCEPTION_INVALID_DISPOSITION:      return L"EXCEPTION_INVALID_DISPOSITION";
    case EXCEPTION_NONCONTINUABLE_EXCEPTION: return L"EXCEPTION_NONCONTINUABLE_EXCEPTION";
    case EXCEPTION_PRIV_INSTRUCTION:         return L"EXCEPTION_PRIV_INSTRUCTION";
    case EXCEPTION_STACK_OVERFLOW:           return L"EXCEPTION_STACK_OVERFLOW";
    case 0xc0000374:                         return L"STATUS_HEAP_CORRUPTION";
    case 0xc0000409:                         return L"STATUS_STACK_BUFFER_OVERRUN";
    case 0xe06d7363:                         return L"C++ exception";
    case FatalErrorExceptionCode:            return L"Error detected by the runtime library";
    default:                                 return L"Unknown";
    }
}


void putExceptionInformation(TextBuffer& text, EXCEPTION_POINTERS* exceptionInfo)
{
    auto record = exceptionInfo->ExceptionRecord;

    text.put(L"Exception code  : ");
    text.putHex(record->ExceptionCode, 8);
    text.put(L" (");
    text.put(getExceptionCodeName(record->ExceptionCode));
    text.put(L")\r\n");

    if(record->ExceptionCode == FatalErrorExceptionCode){
        /*
          The exception record has been created in the handler of this module and the
          address does not indicate the position where the error occurred. The position can
          be obtained from the call stack recorded in the dump file.
        */
        return;
    }

    text.put(L"Exception addr  : ");
    text.putPointer(record->ExceptionAddress);
    text.put(L"\r\n");

    /*
      The following two items correspond to the faulting module name and the fault offset
      recorded by Windows Error Reporting in the application error event of the event log.
    */
    HMODULE module = nullptr;
    if(GetModuleHandleExW(
           GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
           reinterpret_cast<LPCWSTR>(record->ExceptionAddress), &module)){
        wchar_t moduleFile[PathBufferSize];
        if(GetModuleFileNameW(module, moduleFile, PathBufferSize)){
            text.put(L"Faulting module : ");
            text.put(moduleFile);
            text.put(L"\r\n");
        }
        text.put(L"Fault offset    : ");
        text.putHex(
            reinterpret_cast<ULONG_PTR>(record->ExceptionAddress) -
            reinterpret_cast<ULONG_PTR>(module),
            16);
        text.put(L"\r\n");
    }

    if((record->ExceptionCode == EXCEPTION_ACCESS_VIOLATION ||
        record->ExceptionCode == EXCEPTION_IN_PAGE_ERROR) &&
       record->NumberParameters >= 2){
        text.put(L"Access type     : ");
        switch(record->ExceptionInformation[0]){
        case 0:  text.put(L"read from ");     break;
        case 1:  text.put(L"write to ");      break;
        case 8:  text.put(L"execution of ");  break;
        default: text.put(L"access to ");     break;
        }
        text.putHex(record->ExceptionInformation[1], 16);
        text.put(L"\r\n");
    }
}


/**
   Get the file name of the module that contains the address and the offset of the address
   in it. The values correspond to the faulting module name and the fault offset recorded
   by Windows Error Reporting in the application error event of the event log.

   \return The module handle, or nullptr if the address does not belong to a loaded module.
*/
HMODULE getModuleOfAddress(ULONG64 address, wchar_t* file, DWORD fileSize, ULONG64& offset)
{
    HMODULE module = nullptr;
    if(!GetModuleHandleExW(
           GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
           reinterpret_cast<LPCWSTR>(address), &module)){
        return nullptr;
    }
    offset = address - reinterpret_cast<ULONG64>(module);
    if(file){
        file[0] = L'\0';
        GetModuleFileNameW(module, file, fileSize);
    }
    return module;
}


const wchar_t* getFileNamePart(const wchar_t* path)
{
    const wchar_t* name = path;
    for(const wchar_t* p = path; *p; ++p){
        if(*p == L'\\' || *p == L'/'){
            name = p + 1;
        }
    }
    return name;
}


/**
   Trace back the call stack of the crashed thread.

   The unwind information of each function is embedded in the module itself on x64 and
   ARM64, and the modules are still mapped in the process, so the call stack can be
   obtained in the process without any external file. Note that the functions used here
   do not allocate any memory.

   \return The number of the obtained frames.
*/
int captureStackFrames(const CONTEXT& exceptionContext, ULONG64* frames, int maxFrames)
{
#if defined(_M_X64) || defined(_M_ARM64)
    unwindContext = exceptionContext;
    int numFrames = 0;

    while(numFrames < maxFrames){
#if defined(_M_X64)
        ULONG64 pc = unwindContext.Rip;
#else
        ULONG64 pc = unwindContext.Pc;
#endif
        if(pc == 0){
            break;
        }
        frames[numFrames++] = pc;

        DWORD64 imageBase = 0;
        auto functionEntry = RtlLookupFunctionEntry(pc, &imageBase, nullptr);
        if(functionEntry){
            PVOID handlerData = nullptr;
            ULONG64 establisherFrame = 0;
            RtlVirtualUnwind(
                UNW_FLAG_NHANDLER, imageBase, pc, functionEntry, &unwindContext,
                &handlerData, &establisherFrame, nullptr);
        } else {
            /*
              A leaf function does not have unwind information. The return address is at
              the top of the stack in that case.
            */
#if defined(_M_X64)
            if(unwindContext.Rsp == 0){
                break;
            }
            unwindContext.Rip = *reinterpret_cast<ULONG64*>(unwindContext.Rsp);
            unwindContext.Rsp += sizeof(ULONG64);
#else
            if(unwindContext.Lr == pc){
                break;
            }
            unwindContext.Pc = unwindContext.Lr;
#endif
        }
#if defined(_M_X64)
        if(unwindContext.Rip == pc){
#else
        if(unwindContext.Pc == pc){
#endif
            break; // The unwinding does not make progress
        }
    }

    return numFrames;

#else
    return 0;
#endif
}


void putStackFrames(TextBuffer& text, int numFrames, bool doResolveSymbols)
{
    static struct {
        SYMBOL_INFOW info;
        wchar_t nameBuffer[MaxSymbolNameLength];
    } symbol;

    HANDLE process = GetCurrentProcess();
    bool isAnyDebugInfoMissing = false;

    /**
       Check whether the symbols of the module that contains the address are read from its
       debug information. A symbol obtained without it is the exported symbol nearest to
       the address, which is often a function other than the one that actually contains the
       address. Such a name must be distinguished so that it is not mistaken for the exact
       one.
    */
    auto isDebugInfoAvailable =
        [process](ULONG64 address) -> bool {
            static IMAGEHLP_MODULEW64 moduleInfo;
            ZeroMemory(&moduleInfo, sizeof(moduleInfo));
            moduleInfo.SizeOfStruct = sizeof(moduleInfo);
            if(!symGetModuleInfoW64(process, address, &moduleInfo)){
                return false;
            }
            return moduleInfo.SymType == SymPdb || moduleInfo.SymType == SymDia;
        };

    for(int i = 0; i < numFrames; ++i){
        ULONG64 address = stackFrames[i];
        text.put(L"  ");
        text.putUInt(i, 2);
        text.put(L"  ");

        wchar_t moduleFile[PathBufferSize];
        ULONG64 offset = 0;
        if(getModuleOfAddress(address, moduleFile, PathBufferSize, offset)){
            text.putPadded(getFileNamePart(moduleFile), 26);
            text.put(L"+ ");
            text.putHex(offset, 8);
        } else {
            text.putPadded(L"(unknown module)", 26);
            text.put(L"  ");
            text.putHex(address, 16);
        }

        if(symFromAddrW && doResolveSymbols){
            symbol.info.SizeOfStruct = sizeof(SYMBOL_INFOW);
            symbol.info.MaxNameLen = MaxSymbolNameLength;
            DWORD64 displacement = 0;
            if(symFromAddrW(process, address, &displacement, &symbol.info)){
                bool hasDebugInfo = isDebugInfoAvailable(address);
                text.put(L"  ");
                text.put(symbol.info.Name);
                text.put(L" + ");
                text.putHexValue(displacement);

                if(hasDebugInfo){
                    IMAGEHLP_LINEW64 line;
                    ZeroMemory(&line, sizeof(line));
                    line.SizeOfStruct = sizeof(line);
                    DWORD lineDisplacement = 0;
                    if(symGetLineFromAddrW64(process, address, &lineDisplacement, &line)){
                        text.put(L"  [");
                        text.put(getFileNamePart(line.FileName));
                        text.put(L":");
                        text.putUInt(line.LineNumber);
                        text.put(L"]");
                    }
                } else {
                    text.put(L"  (*)");
                    isAnyDebugInfoMissing = true;
                }
            } else {
                isAnyDebugInfoMissing = true;
            }
        }
        text.put(L"\r\n");
    }

    if(isAnyDebugInfoMissing){
        text.put(L"\r\n"
                 L"  The debug information (PDB file) of the modules of the frames without a\r\n"
                 L"  source line is not available on this computer. A name marked with (*) is\r\n"
                 L"  just the exported symbol nearest to the address and is often a function\r\n"
                 L"  other than the one that actually contains it. The module name and the\r\n"
                 L"  offset in it are always exact, and the developer can identify the exact\r\n"
                 L"  position from them with the same binary and its debug information.\r\n");
    }
}


void putStackTrace(TextBuffer& text, const CONTEXT& context, DWORD exceptionCode)
{
    /*
      The symbol handler allocates memory, which must not be done when the heap of the
      process is known to be broken. The frames are still listed with the module names and
      the offsets in them in that case.
    */
    bool doResolveSymbols = (exceptionCode != STATUS_HEAP_CORRUPTION);

    /*
      The functions used here are not guaranteed to work in a broken process because the
      symbol handler allocates memory. The report has already been composed up to this
      point, so a failure here only makes the call stack missing from the report.
    */
    __try {
        int numFrames = captureStackFrames(context, stackFrames, MaxNumStackFrames);
        if(numFrames == 0){
            text.put(L"  (not available)\r\n");
        } else {
            if(symInitializeW && doResolveSymbols){
                /*
                  The search path is given explicitly so that the symbol handler does not
                  use the _NT_SYMBOL_PATH environment variable, which may refer to a symbol
                  server on the network and make the crash handling take a long time. Note
                  that the PDB file of a module is found through the path embedded in the
                  module itself even if it is not in the search path, which is usually the
                  case for the modules built on the computer where the crash occurred.
                */
                wchar_t searchPath[PathBufferSize];
                TextBuffer path(searchPath, PathBufferSize);
                path.put(executableFileW);
                size_t n = getFileNamePart(searchPath) - searchPath;
                if(n > 0){
                    searchPath[n - 1] = L'\0';
                }
                symSetOptions(
                    SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS | SYMOPT_LOAD_LINES |
                    SYMOPT_FAIL_CRITICAL_ERRORS | SYMOPT_NO_PROMPTS);
                symInitializeW(GetCurrentProcess(), searchPath, TRUE);
            }
            putStackFrames(text, numFrames, symInitializeW && doResolveSymbols);
            if(symCleanup){
                symCleanup(GetCurrentProcess());
            }
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        text.put(L"  (the call stack could not be obtained)\r\n");
    }
}


/**
   Put the list of the loaded modules. The list allows the developer to check whether a
   third-party module such as a security or overlay software is injected into the process,
   and to identify the exact build of each module by its time stamp and size.
*/
void putModuleList(TextBuffer& text)
{
    if(!enumProcessModules){
        return;
    }
    __try {
        static HMODULE modules[MaxNumModules];
        DWORD sizeNeeded = 0;
        if(!enumProcessModules(
               GetCurrentProcess(), modules, sizeof(modules), &sizeNeeded)){
            return;
        }
        int numModules = static_cast<int>(sizeNeeded / sizeof(HMODULE));
        if(numModules > MaxNumModules){
            numModules = MaxNumModules;
        }

        text.put(L"\r\nLoaded modules (");
        text.putUInt(numModules);
        text.put(L"):\r\n\r\n");
        text.put(L"  base address        size      time stamp  file\r\n");

        for(int i = 0; i < numModules; ++i){
            auto base = reinterpret_cast<const unsigned char*>(modules[i]);
            DWORD timeStamp = 0;
            DWORD imageSize = 0;
            auto dosHeader = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
            if(dosHeader->e_magic == IMAGE_DOS_SIGNATURE){
                auto ntHeaders =
                    reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dosHeader->e_lfanew);
                if(ntHeaders->Signature == IMAGE_NT_SIGNATURE){
                    timeStamp = ntHeaders->FileHeader.TimeDateStamp;
                    imageSize = ntHeaders->OptionalHeader.SizeOfImage;
                }
            }
            text.put(L"  ");
            text.putHex(reinterpret_cast<ULONG64>(base), 16);
            text.put(L"  ");
            text.putHex(imageSize, 8);
            text.put(L"  ");
            text.putHex(timeStamp, 8);
            text.put(L"  ");
            wchar_t moduleFile[PathBufferSize];
            moduleFile[0] = L'\0';
            GetModuleFileNameW(modules[i], moduleFile, PathBufferSize);
            text.put(moduleFile);
            text.put(L"\r\n");
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        text.put(L"  (the module list could not be obtained)\r\n");
    }
}


/**
   Write a text file that summarizes the crash. The file contains the call stack of the
   crashed thread and is intended to be the primary information the user reports. The
   developer can see where the application crashed from this file alone when the debug
   information of the modules is available on the computer where the crash occurred.
*/
void writeReportFile(
    const wchar_t* reportFile, const wchar_t* dumpFile, const SYSTEMTIME& time,
    EXCEPTION_POINTERS* exceptionInfo, const wchar_t* reason)
{
    TextBuffer text(reportBuffer, ReportBufferSize);

    text.put(L"Crash report of an application based on Choreonoid\r\n\r\n");
    text.put(L"Application     : ");
    text.put(notificationCaptionW);
    text.put(L"\r\n");
    text.put(L"Choreonoid ver. : ");
    text.putAscii(CNOID_FULL_VERSION_STRING);
    text.put(L"\r\n");
    text.put(L"Executable      : ");
    text.put(executableFileW);
    text.put(L"\r\n");
    text.put(L"Date            : ");
    text.putDateTime(time, true);
    text.put(L"\r\n");
    text.put(L"Process ID      : ");
    text.putUInt(GetCurrentProcessId());
    text.put(L"\r\n");
    text.put(L"Thread ID       : ");
    text.putUInt(GetCurrentThreadId());
    text.put(L"\r\n");
    text.put(L"Reason          : ");
    text.put(reason);
    text.put(L"\r\n");

    putExceptionInformation(text, exceptionInfo);

    text.put(L"Dump file       : ");
    text.put(dumpFile);
    text.put(L"\r\n");
    text.put(L"Dump type       : ");
    text.put(isFullDumpEnabled ? L"full dump" : L"minidump");
    text.put(L"\r\n\r\n");

    text.put(L"Call stack of the crashed thread (the most recent call first):\r\n\r\n");
    putStackTrace(text, *exceptionInfo->ContextRecord,
                  exceptionInfo->ExceptionRecord->ExceptionCode);

    putModuleList(text);

    text.put(L"\r\nPlease send this file to the developer of the application. The dump file\r\n"
             L"listed above is also useful to investigate the details of the problem.\r\n");

    HANDLE file = CreateFileW(
        reportFile, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if(file == INVALID_HANDLE_VALUE){
        return;
    }
    int size = WideCharToMultiByte(
        CP_UTF8, 0, text.text(), static_cast<int>(text.size()),
        reportUtf8Buffer, sizeof(reportUtf8Buffer), nullptr, nullptr);
    if(size > 0){
        DWORD written;
        // The byte order mark is added so that the encoding is not mistaken by a text editor
        WriteFile(file, "\xef\xbb\xbf", 3, &written, nullptr);
        WriteFile(file, reportUtf8Buffer, static_cast<DWORD>(size), &written, nullptr);
    }
    CloseHandle(file);
}


void putMessageToStandardError(const wchar_t* message)
{
    HANDLE handle = GetStdHandle(STD_ERROR_HANDLE);
    if(!handle || handle == INVALID_HANDLE_VALUE){
        return;
    }
    char utf8[MessageBufferSize * 3];
    int size = WideCharToMultiByte(CP_UTF8, 0, message, -1, utf8, sizeof(utf8), nullptr, nullptr);
    if(size > 1){
        DWORD written;
        WriteFile(handle, utf8, static_cast<DWORD>(size - 1), &written, nullptr);
        WriteFile(handle, "\r\n", 2, &written, nullptr);
    }
}


void notifyCrash(const wchar_t* dumpFile)
{
    wchar_t buf[MessageBufferSize];
    TextBuffer message(buf, MessageBufferSize);
    message.put(notificationPrefixW);
    message.put(dumpFile);
    message.put(notificationSuffixW);

    putMessageToStandardError(message.text());

    if(notificationDialogEnabled){
        MessageBoxW(
            nullptr, message.text(), notificationCaptionW,
            MB_OK | MB_ICONERROR | MB_SETFOREGROUND | MB_TASKMODAL);
    }
}


bool writeDumpFile(const wchar_t* dumpFile, EXCEPTION_POINTERS* exceptionInfo)
{
    HANDLE file = CreateFileW(
        dumpFile, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if(file == INVALID_HANDLE_VALUE){
        return false;
    }
    MINIDUMP_EXCEPTION_INFORMATION info;
    info.ThreadId = GetCurrentThreadId();
    info.ExceptionPointers = exceptionInfo;
    // The exception information is in the address space of this process
    info.ClientPointers = FALSE;

    MINIDUMP_TYPE type;
    if(isFullDumpEnabled){
        type = static_cast<MINIDUMP_TYPE>(
            MiniDumpWithFullMemory | MiniDumpWithFullMemoryInfo | MiniDumpWithHandleData |
            MiniDumpWithUnloadedModules | MiniDumpWithThreadInfo);
    } else {
        /*
          The memory blocks referenced by the values on the stacks are included in addition
          to the stacks themselves so that the objects related to the crash can also be
          inspected. The file size is still usually less than a few megabytes.
        */
        type = static_cast<MINIDUMP_TYPE>(
            MiniDumpNormal | MiniDumpWithUnloadedModules | MiniDumpWithThreadInfo |
            MiniDumpWithIndirectlyReferencedMemory | MiniDumpScanMemory);
    }

    BOOL result = miniDumpWriteDump(
        GetCurrentProcess(), GetCurrentProcessId(), file, type, &info, nullptr, nullptr);

    CloseHandle(file);

    if(!result){
        DeleteFileW(dumpFile);
    }

    return result;
}


/**
   \return True if the dump file has been written.
*/
bool processCrash(EXCEPTION_POINTERS* exceptionInfo, const wchar_t* reason)
{
    if(!installed || !miniDumpWriteDump){
        return false;
    }
    if(InterlockedIncrement(&crashCounter) > 1){
        /*
          Another thread is writing a dump file, or this handler itself has crashed. In the
          former case the process is terminated by the thread that is writing the dump file
          and this thread just has to wait for it. The wait is limited so that the process
          does not hang up when the dump writing does not finish.
        */
        Sleep(60000);
        return false;
    }

    SYSTEMTIME time;
    GetLocalTime(&time);

    wchar_t basePath[PathBufferSize];
    TextBuffer base(basePath, PathBufferSize);
    base.put(dumpDirectoryW);
    base.put(L'\\');
    base.put(fileBaseNameW);
    base.put(L'_');
    base.putDateTime(time, false);
    base.put(L'_');
    base.putUInt(GetCurrentProcessId());

    wchar_t dumpFile[PathBufferSize];
    TextBuffer dumpPath(dumpFile, PathBufferSize);
    dumpPath.put(basePath);
    dumpPath.put(L".dmp");

    if(!writeDumpFile(dumpFile, exceptionInfo)){
        return false;
    }

    wchar_t reportFile[PathBufferSize];
    TextBuffer reportPath(reportFile, PathBufferSize);
    reportPath.put(basePath);
    reportPath.put(L".txt");

    writeReportFile(reportFile, dumpFile, time, exceptionInfo, reason);

    // The report file is notified because it is the file to be read and reported first
    notifyCrash(reportFile);

    return true;
}


LONG WINAPI unhandledExceptionFilter(EXCEPTION_POINTERS* exceptionInfo)
{
    if(IsDebuggerPresent()){
        return EXCEPTION_CONTINUE_SEARCH;
    }
    processCrash(exceptionInfo, L"Unhandled exception");

    /*
      The default processing of the system is not suppressed even when the dump file has
      been written, so that the application error event is still recorded in the event log
      and the local dump function of Windows Error Reporting also works when it is enabled.
      The process is terminated by the default processing.
    */
    if(previousExceptionFilter){
        return previousExceptionFilter(exceptionInfo);
    }
    return EXCEPTION_CONTINUE_SEARCH;
}


/**
   Write a dump for a fatal error that is not raised as a structured exception. The context
   of the current thread is captured so that the call stack that leads to the error can also
   be obtained from the dump file.
*/
void processFatalError(const wchar_t* reason)
{
    if(IsDebuggerPresent()){
        return;
    }
    CONTEXT context;
    RtlCaptureContext(&context);

    EXCEPTION_RECORD record;
    ZeroMemory(&record, sizeof(record));
    record.ExceptionCode = FatalErrorExceptionCode;
    record.ExceptionFlags = EXCEPTION_NONCONTINUABLE;
#if defined(_M_X64)
    record.ExceptionAddress = reinterpret_cast<PVOID>(context.Rip);
#elif defined(_M_IX86)
    record.ExceptionAddress = reinterpret_cast<PVOID>(context.Eip);
#elif defined(_M_ARM64)
    record.ExceptionAddress = reinterpret_cast<PVOID>(context.Pc);
#endif

    EXCEPTION_POINTERS exceptionInfo;
    exceptionInfo.ExceptionRecord = &record;
    exceptionInfo.ContextRecord = &context;

    processCrash(&exceptionInfo, reason);

    // The process must not continue running because the runtime library is in a fatal state
    TerminateProcess(GetCurrentProcess(), 3);
}


void onAbortSignal(int)
{
    processFatalError(L"Abnormal termination by the abort function");
}


void onTerminate()
{
    processFatalError(L"An exception that is not caught by any handler");
}


#ifdef _MSC_VER

void onPureCall()
{
    processFatalError(L"Call of a pure virtual function");
}


void onInvalidParameter(
    const wchar_t* /* expression */, const wchar_t* /* function */, const wchar_t* /* file */,
    unsigned int /* line */, uintptr_t /* reserved */)
{
    processFatalError(L"An invalid parameter given to a runtime library function");
}

#endif


bool setUpDumpDirectory(const std::string& applicationName, const std::string& organizationName)
{
    TextBuffer directory(dumpDirectoryW, PathBufferSize);

    wchar_t value[PathBufferSize];
    if(getEnvironmentValue(L"CNOID_CRASH_DUMP_DIR", value, PathBufferSize)){
        directory.put(value);
    } else {
        if(getEnvironmentValue(L"LOCALAPPDATA", value, PathBufferSize)){
            directory.put(value);
        } else if(getEnvironmentValue(L"TEMP", value, PathBufferSize)){
            directory.put(value);
        } else {
            return false;
        }
        auto organization = toFileNameString(organizationName);
        if(!organization.empty()){
            directory.put(L'\\');
            directory.put(organization.c_str());
        }
        auto application = toFileNameString(applicationName);
        if(!application.empty()){
            directory.put(L'\\');
            directory.put(application.c_str());
        }
        directory.put(L"\\CrashDumps");
    }

    // Remove the trailing separators so that the paths are composed consistently
    size_t length = directory.size();
    while(length > 0 && dumpDirectoryW[length - 1] == L'\\'){
        dumpDirectoryW[--length] = L'\0';
    }
    if(length == 0){
        return false;
    }
    if(!createDirectories(dumpDirectoryW)){
        return false;
    }
    return true;
}


void setUpFileBaseName()
{
    GetModuleFileNameW(nullptr, executableFileW, PathBufferSize);

    const wchar_t* begin = executableFileW;
    const wchar_t* end = executableFileW;
    for(const wchar_t* p = executableFileW; *p; ++p){
        if(*p == L'\\' || *p == L'/'){
            begin = p + 1;
        }
        end = p + 1;
    }
    for(const wchar_t* p = end; p > begin; --p){
        if(*(p - 1) == L'.'){
            end = p - 1;
            break;
        }
    }
    TextBuffer baseName(fileBaseNameW, 128);
    for(const wchar_t* p = begin; p < end; ++p){
        baseName.put(*p);
    }
    if(baseName.size() == 0){
        baseName.put(L"crash");
    }
}


bool loadDbgHelpLibrary()
{
    /*
      The library is loaded and the function is resolved in advance because the loader
      cannot always be used safely after a crash. Note that the library is explicitly
      loaded from the system directory so that a library file with the same name placed
      in the application directory is not loaded by mistake.
    */
    HMODULE dbgHelp = LoadLibraryExW(L"dbghelp.dll", nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32);
    if(!dbgHelp){
        dbgHelp = LoadLibraryW(L"dbghelp.dll");
    }
    if(!dbgHelp){
        return false;
    }
    miniDumpWriteDump =
        reinterpret_cast<MiniDumpWriteDumpFunc>(GetProcAddress(dbgHelp, "MiniDumpWriteDump"));

    /*
      The following functions are used to put the function names and the source lines into
      the report. They are optional because the report is still useful without them.
    */
    symSetOptions = reinterpret_cast<SymSetOptionsFunc>(GetProcAddress(dbgHelp, "SymSetOptions"));
    symInitializeW = reinterpret_cast<SymInitializeWFunc>(GetProcAddress(dbgHelp, "SymInitializeW"));
    symCleanup = reinterpret_cast<SymCleanupFunc>(GetProcAddress(dbgHelp, "SymCleanup"));
    symFromAddrW = reinterpret_cast<SymFromAddrWFunc>(GetProcAddress(dbgHelp, "SymFromAddrW"));
    symGetLineFromAddrW64 = reinterpret_cast<SymGetLineFromAddrW64Func>(
        GetProcAddress(dbgHelp, "SymGetLineFromAddrW64"));
    symGetModuleInfoW64 = reinterpret_cast<SymGetModuleInfoW64Func>(
        GetProcAddress(dbgHelp, "SymGetModuleInfoW64"));
    if(!symSetOptions || !symInitializeW || !symFromAddrW || !symGetLineFromAddrW64 ||
       !symGetModuleInfoW64){
        symFromAddrW = nullptr;
    }

    if(auto kernel32 = GetModuleHandleW(L"kernel32.dll")){
        enumProcessModules = reinterpret_cast<EnumProcessModulesFunc>(
            GetProcAddress(kernel32, "K32EnumProcessModules"));
    }

    return miniDumpWriteDump != nullptr;
}

}


bool WindowsCrashDumpHandler::install
(const std::string& applicationName_, const std::string& organizationName, bool isFullDumpEnabled_)
{
    if(installed){
        return true;
    }
    if(!loadDbgHelpLibrary()){
        return false;
    }
    if(!setUpDumpDirectory(applicationName_, organizationName)){
        return false;
    }

    applicationName = applicationName_;
    isFullDumpEnabled = isFullDumpEnabled_;
    setUpFileBaseName();
    removeOldDumpFiles();
    updateNotificationDialogMessage();

    /*
      Reserve the stack area that is used when the stack has overflowed so that a dump file
      can also be written for a stack overflow. Note that the reservation is only effective
      for the thread that executes this function.
    */
    ULONG stackSize = 64 * 1024;
    SetThreadStackGuarantee(&stackSize);

    previousExceptionFilter = SetUnhandledExceptionFilter(unhandledExceptionFilter);

    /*
      The following errors detected by the runtime library do not raise a structured
      exception and are not passed to the unhandled exception filter. They are captured
      separately so that a dump file is also written for them.
    */
    signal(SIGABRT, onAbortSignal);
    std::set_terminate(onTerminate);
#ifdef _MSC_VER
    _set_purecall_handler(onPureCall);
    _set_invalid_parameter_handler(onInvalidParameter);
#endif

    installed = true;

    return true;
}


void WindowsCrashDumpHandler::setNotificationDialogEnabled(bool on)
{
    notificationDialogEnabled = on;
}


void WindowsCrashDumpHandler::updateNotificationDialogMessage()
{
    if(dumpDirectoryW[0] == L'\0'){
        return;
    }

    TextBuffer caption(notificationCaptionW, 256);
    caption.put(toWideString(applicationName).c_str());

    /*
      The message is composed of the parts before and after the dump file path because the
      path is only determined when a crash occurs and the message must be prepared in
      advance. The parts are obtained by splitting the translated message at the position
      of the path, which is temporarily filled with a control character that never appears
      in a message.
    */
    static const char* pathMark = "\x1f";

    auto message =
        toWideString(
            formatR(
                _("{0} has stopped unexpectedly.\n\n"
                  "The information on the crash has been saved in the following file:\n\n"
                  "{1}\n\n"
                  "Sending the file to the developer of the application may help identify "
                  "the cause of the problem."),
                applicationName, pathMark));

    TextBuffer prefix(notificationPrefixW, MessageBufferSize);
    TextBuffer suffix(notificationSuffixW, MessageBufferSize);

    auto position = message.find(L'\x1f');
    if(position == std::wstring::npos){
        prefix.put(message.c_str());
    } else {
        suffix.put(message.c_str() + position + 1);
        message.resize(position);
        prefix.put(message.c_str());
    }
}
