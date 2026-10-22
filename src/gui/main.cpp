#include <list>
#include <QApplication>
#include <QSharedPointer>
#include <QMessageBox>
#include "camera_window.hpp"
#include "camera_facade.hpp"
#include "affinity.hpp"
#include <iostream>
#include <cstdio>
#include <QCommandLineParser>

#ifdef _WIN32
#include <windows.h>
#include <dbghelp.h>
#include <io.h>
#include <fcntl.h>
#include <intrin.h>
#include <csignal>
#include <cstdlib>
#include <exception>
#endif

// ------------------------------------------------------------------------
// Crash diagnostics (Windows only): everything written to stdout/stderr is
// also saved to <exe dir>\logs\bias_console_<timestamp>.log, and an
// unhandled exception or CRT error (abort, std::terminate, pure virtual
// call, invalid CRT parameter) writes <exe dir>\logs\crash_<timestamp>.dmp.
// ------------------------------------------------------------------------
namespace {

#ifdef _WIN32

    // Resolved once at startup. The crash handler must not touch the heap (it
    // may be what is corrupted), so it only uses these globals plus the stack.
    wchar_t g_logDir[MAX_PATH] = L".";
    HANDLE g_logFile = INVALID_HANDLE_VALUE;
    HANDLE g_consoleOut = INVALID_HANDLE_VALUE;
    HANDLE g_logPipeRead = INVALID_HANDLE_VALUE;
    HANDLE g_logPipeWrite = INVALID_HANDLE_VALUE;

    // Id of the thread that is reporting a crash, 0 while none is
    volatile LONG g_crashThreadId = 0;

    // Exception codes (customer bit set, "B1A5") that identify the CRT error
    // path in the minidump
    const DWORD CRASH_CODE_ABORT = 0xE0B1A501;
    const DWORD CRASH_CODE_TERMINATE = 0xE0B1A502;
    const DWORD CRASH_CODE_PURE_CALL = 0xE0B1A503;
    const DWORD CRASH_CODE_INVALID_PARAMETER = 0xE0B1A504;

    void initLogDir()
    {
        wchar_t exePath[MAX_PATH];
        DWORD len = GetModuleFileNameW(nullptr, exePath, MAX_PATH);
        if ((len == 0) || (len >= MAX_PATH))
        {
            return;
        }
        wchar_t *lastSlash = wcsrchr(exePath, L'\\');
        if (lastSlash != nullptr)
        {
            *lastSlash = L'\0';
        }
        swprintf_s(g_logDir, L"%s\\logs", exePath);
        CreateDirectoryW(g_logDir, nullptr);
    }

    void writeAll(HANDLE h, const char *buf, DWORD n)
    {
        DWORD written = 0;
        if (h != INVALID_HANDLE_VALUE)
        {
            WriteFile(h, buf, n, &written, nullptr);
        }
    }

    DWORD WINAPI logReaderThread(LPVOID)
    {
        char buf[4096];
        DWORD n = 0;
        while (ReadFile(g_logPipeRead, buf, sizeof(buf), &n, nullptr) && (n > 0))
        {
            writeAll(g_consoleOut, buf, n);
            writeAll(g_logFile, buf, n);
        }
        return 0;
    }

    // Tees stdout and stderr (file descriptors 1 and 2) into a pipe whose
    // reader thread copies everything to the original console and to the log
    // file. Working at the descriptor level captures printf, std::cout,
    // qDebug/Qt runtime warnings and the camera SDK's own output alike, in
    // order. Console scrollback is gone once the process crashes, so without
    // this the output immediately preceding a crash is lost.
    void installConsoleLogFile()
    {
        SYSTEMTIME st;
        GetLocalTime(&st);
        wchar_t logPath[MAX_PATH];
        swprintf_s(
                logPath,
                L"%s\\bias_console_%04d%02d%02d_%02d%02d%02d.log",
                g_logDir, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond
                );

        g_logFile = CreateFileW(
                logPath, GENERIC_WRITE, FILE_SHARE_READ, nullptr,
                CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr
                );
        if (g_logFile == INVALID_HANDLE_VALUE)
        {
            return;
        }

        HANDLE pipeWrite = INVALID_HANDLE_VALUE;
        if (!CreatePipe(&g_logPipeRead, &pipeWrite, nullptr, 1 << 20))
        {
            CloseHandle(g_logFile);
            g_logFile = INVALID_HANDLE_VALUE;
            return;
        }

        // Private copy of the original console handle for the reader thread to echo to
        int consoleFd = _dup(_fileno(stdout));
        if (consoleFd != -1)
        {
            g_consoleOut = (HANDLE)_get_osfhandle(consoleFd);
        }

        // Unbuffered so every write reaches the pipe (and the file) immediately
        setvbuf(stdout, nullptr, _IONBF, 0);
        setvbuf(stderr, nullptr, _IONBF, 0);

        int pipeWriteFd = _open_osfhandle((intptr_t)pipeWrite, _O_WRONLY | _O_TEXT);
        _dup2(pipeWriteFd, _fileno(stdout));
        _dup2(pipeWriteFd, _fileno(stderr));
        _close(pipeWriteFd);
        g_logPipeWrite = (HANDLE)_get_osfhandle(_fileno(stderr));

        HANDLE thread = CreateThread(nullptr, 0, logReaderThread, nullptr, 0, nullptr);
        if (thread != nullptr)
        {
            CloseHandle(thread);
        }

        printf("Logging console output to: %ls\n", logPath);
    }

    // Route Qt's own messages (qDebug/qWarning and Qt's internal runtime
    // warnings such as cross-thread QObject misuse) to stderr explicitly, so
    // they land in the log regardless of Qt's console-detection heuristics.
    void qtMessageHandler(QtMsgType type, const QMessageLogContext &, const QString &msg)
    {
        const char *prefix = "Qt debug";
        switch (type)
        {
            case QtInfoMsg:     prefix = "Qt info";     break;
            case QtWarningMsg:  prefix = "Qt warning";  break;
            case QtCriticalMsg: prefix = "Qt critical"; break;
            case QtFatalMsg:    prefix = "Qt fatal";    break;
            default: break;
        }
        fprintf(stderr, "%s: %s\n", prefix, qPrintable(msg));
        if (type == QtFatalMsg)
        {
            abort();
        }
    }

    // Marks this thread as the one reporting the crash. Returns false if it
    // already is (e.g. std::terminate followed by its abort()). Any other
    // thread that crashes meanwhile parks here until the process is gone, so
    // it cannot end the process halfway through the first thread's dump.
    bool beginCrashReport()
    {
        LONG self = (LONG)GetCurrentThreadId();
        LONG owner = InterlockedCompareExchange(&g_crashThreadId, self, 0);
        if (owner == 0)
        {
            return true;
        }
        if (owner != self)
        {
            Sleep(INFINITE);
        }
        return false;
    }

    // Writes a crash marker to the console and the log, and a minidump, so
    // the faulting thread/call can be identified in a debugger. reason is
    // nullptr for an unhandled exception, otherwise it names the CRT error.
    void reportCrash(EXCEPTION_POINTERS *exceptionPointers, const char *reason)
    {
        if (!beginCrashReport())
        {
            return;
        }

        SYSTEMTIME st;
        GetLocalTime(&st);
        wchar_t dumpPath[MAX_PATH];
        swprintf_s(
                dumpPath,
                L"%s\\crash_%04d%02d%02d_%02d%02d%02d.dmp",
                g_logDir, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond
                );

        char marker[1024];
        int len = 0;
        if (reason != nullptr)
        {
            len = sprintf_s(
                    marker,
                    "\r\nCRASH: %s in thread %lu, see %ls\r\n",
                    reason, GetCurrentThreadId(), dumpPath
                    );
        }
        else
        {
            DWORD code = 0;
            void *addr = nullptr;
            if ((exceptionPointers != nullptr) && (exceptionPointers->ExceptionRecord != nullptr))
            {
                code = exceptionPointers->ExceptionRecord->ExceptionCode;
                addr = exceptionPointers->ExceptionRecord->ExceptionAddress;
            }
            wchar_t modulePath[MAX_PATH] = L"?";
            uintptr_t moduleOffset = 0;
            HMODULE module = nullptr;
            if (GetModuleHandleExW(
                        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                        (LPCWSTR)addr, &module))
            {
                GetModuleFileNameW(module, modulePath, MAX_PATH);
                moduleOffset = (uintptr_t)addr - (uintptr_t)module;
            }
            len = sprintf_s(
                    marker,
                    "\r\nCRASH: unhandled exception 0x%08lX in thread %lu at %p (%ls + 0x%llX), see %ls\r\n",
                    code, GetCurrentThreadId(), addr, modulePath, (unsigned long long)moduleOffset, dumpPath
                    );
        }

        if (g_logPipeWrite != INVALID_HANDLE_VALUE)
        {
            // Through the pipe, so the marker follows all earlier output in
            // both the console and the log file. The reader thread is still
            // running and copies it within milliseconds; give it time to do
            // so. Do not wait with PeekNamedPipe: the pipe handle is
            // synchronous, so a peek queues behind the reader's blocked
            // ReadFile and never returns.
            if (len > 0)
            {
                writeAll(g_logPipeWrite, marker, (DWORD)len);
            }
            Sleep(200);
        }
        else if (len > 0)
        {
            writeAll(GetStdHandle(STD_ERROR_HANDLE), marker, (DWORD)len);
        }

        HANDLE hFile = CreateFileW(
                dumpPath, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr
                );
        if (hFile != INVALID_HANDLE_VALUE)
        {
            MINIDUMP_EXCEPTION_INFORMATION mdei;
            mdei.ThreadId = GetCurrentThreadId();
            mdei.ExceptionPointers = exceptionPointers;
            mdei.ClientPointers = FALSE;

            MiniDumpWriteDump(
                    GetCurrentProcess(),
                    GetCurrentProcessId(),
                    hFile,
                    (MINIDUMP_TYPE)(MiniDumpWithFullMemory | MiniDumpWithHandleData | MiniDumpWithThreadInfo),
                    &mdei,
                    nullptr,
                    nullptr
                    );
            CloseHandle(hFile);
        }
    }

    // Unhandled exception filter, e.g. for the access violations seen after
    // long unattended runs. Returns EXCEPTION_CONTINUE_SEARCH so Windows
    // Error Reporting still runs and the Event Viewer entry is still written.
    LONG WINAPI writeCrashDump(EXCEPTION_POINTERS *exceptionPointers)
    {
        reportCrash(exceptionPointers, nullptr);
        return EXCEPTION_CONTINUE_SEARCH;
    }

    // Raises and catches an exception so reportCrash gets exception pointers
    // holding this thread's context, and the dump shows the stack of the CRT
    // error. Must not hold C++ objects because of __try.
    void reportCrtError(DWORD code, const char *reason)
    {
        __try
        {
            RaiseException(code, EXCEPTION_NONCONTINUABLE, 0, nullptr);
        }
        __except (reportCrash(GetExceptionInformation(), reason), EXCEPTION_EXECUTE_HANDLER)
        {
        }
    }

    // The handlers below cover CRT error paths that end the process without
    // reaching the unhandled exception filter.

    // abort() (also reached from qFatal, std::terminate and pure virtual
    // calls) fast-fails straight into Windows Error Reporting once this
    // handler returns.
    void __cdecl onAbortSignal(int)
    {
        reportCrtError(CRASH_CODE_ABORT, "abort() called");
    }

    // std::set_terminate is per thread in the MSVC runtime, so this only
    // labels std::terminate on the main thread. On other threads it still
    // ends in abort() and onAbortSignal.
    void onTerminate()
    {
        reportCrtError(CRASH_CODE_TERMINATE, "std::terminate called");
        abort();
    }

    void __cdecl onPureCall()
    {
        reportCrtError(CRASH_CODE_PURE_CALL, "pure virtual function call");
        abort();
    }

    // Keeps the default behavior of fast-failing after the report
    void __cdecl onInvalidParameter(
            const wchar_t *, const wchar_t *, const wchar_t *, unsigned int, uintptr_t)
    {
        reportCrtError(CRASH_CODE_INVALID_PARAMETER, "invalid parameter passed to a CRT function");
        __fastfail(FAST_FAIL_INVALID_ARG);
    }

    void installCrashDiagnostics()
    {
        initLogDir();
        installConsoleLogFile();
        qInstallMessageHandler(qtMessageHandler);
        SetUnhandledExceptionFilter(writeCrashDump);
        signal(SIGABRT, onAbortSignal);
        std::set_terminate(onTerminate);
        _set_purecall_handler(onPureCall);
        _set_invalid_parameter_handler(onInvalidParameter);
    }

#else

    void installCrashDiagnostics() {}

#endif

} // anonymous namespace


// ------------------------------------------------------------------------
// TO DO ... temporary main function. Currently just opens a camera
// window for each camera found attached to the system.
// ------------------------------------------------------------------------
int main (int argc, char *argv[])
{
    installCrashDiagnostics();

    // reduce number of threads for openCV to avoid temporarily allocating all cores
    cv::setNumThreads(4);

    QApplication app(argc, argv);

    QCoreApplication::setApplicationName("BIAS");

    QCommandLineParser parser;
    parser.setApplicationDescription("BIAS help");
    parser.addHelpOption();
    // -i <in-video-file> or --in <in-video-file> or --in-video <in-video-file> 
    // capture from video instead of camera
    parser.addOption(QCommandLineOption(
        QStringList() << "i" << "in" << "in-video",
        QString("Capture video from file <in-video-file>"),
        QString("in-video-file")));
    // -c <config-file> or --config <config-file>
    parser.addOption(QCommandLineOption(
		QStringList() << "c" << "config",
		QString("Load configuration from <config-file>"),
		QString("config-file")));
    // -s <start-frame> or --start-frame <start-frame>
    // when reading from a video, start tracking from this frame instead of the beginning
    parser.addOption(QCommandLineOption(
		QStringList() << "s" << "start-frame",
		QString("Start tracking from video frame <start-frame> (video input only)"),
		QString("start-frame")));
    // -o <out-track-file> or --out-track <out-track-file>
    // output trajectory file path (overrides the value in the config)
    parser.addOption(QCommandLineOption(
		QStringList() << "o" << "out-track",
		QString("Write the trajectory to <out-track-file> (overrides config)"),
		QString("out-track-file")));
    // --debug-seg-all-frames
    // dump the wing-segmentation debug image for every frame (default: first frame only;
    // requires DEBUG enabled in the config). Run short segments -- one PNG per frame.
    parser.addOption(QCommandLineOption(
		QStringList() << "debug-seg-all-frames",
		QString("Dump wing-segmentation debug image every frame (default: first frame only)")));
    // --play-fps <fps>
    // throttle video playback to <fps> frames/sec (video input only) so it plays at a realistic
    // rate like a real camera; default 0 = flat out, as fast as the machine can decode/track.
    parser.addOption(QCommandLineOption(
		QStringList() << "play-fps",
		QString("Throttle video playback to <fps> (video input only; 0 = flat out)"),
		QString("fps")));

    parser.process(app);
    bias::CmdLineParams params;
    params.inVideoFile = parser.value("in-video");
    params.configFile = parser.value("config");
    params.startFrame = parser.value("start-frame").toInt(); // 0 if not provided
    params.trajectoryFile = parser.value("out-track");
    params.debugSegAllFrames = parser.isSet("debug-seg-all-frames");
    params.playFps = parser.value("play-fps").toDouble(); // 0 if not provided


    bias::GuidList guidList;
    bias::CameraFinder cameraFinder;
    std::list<QSharedPointer<bias::CameraWindow>> windowPtrList;

    if (!params.inVideoFile.isEmpty())
    {
        // Video-input mode: no physical camera. Use a single placeholder guid so one
        // CameraWindow is created; it captures from the input video instead of a camera.
        guidList.push_back(bias::Guid());
    }
    else
    {
        // Get list guids for all cameras found
        try
        {
            guidList = cameraFinder.getGuidList();
        }
        catch (bias::RuntimeError &runtimeError)
        {
            QString msgTitle("Camera Enumeration Error");
            QString msgText("Camera enumeration failed:\n\nError ID: ");
            msgText += QString::number(runtimeError.id());
            msgText += QString("\n\n");
            msgText += QString::fromStdString(runtimeError.what());
            QMessageBox::critical(0, msgTitle,msgText);
            return 0;
        }

        // If no cameras found - error
        if (guidList.empty())
        {
            QString msgTitle("Camera Enumeration Error");
            QString msgText("No cameras found");
            QMessageBox::critical(0, msgTitle,msgText);
            return 0;
        }
    }

    // Get number of cameras
    unsigned int numCam = guidList.size();
    bias::ThreadAffinityService::setNumberOfCameras(numCam);

    // Open camera window for each camera 
    QRect baseGeom;
    QRect nextGeom;
    unsigned int camCnt;
    bias::GuidList::iterator guidIt;
    for (guidIt=guidList.begin(), camCnt=0; guidIt!=guidList.end(); guidIt++, camCnt++)
    {
        bias::Guid guid = *guidIt;
        QSharedPointer<bias::CameraWindow> windowPtr(new bias::CameraWindow(guid, camCnt, numCam, params));
        windowPtr -> show();
        if (camCnt==0)
        {
            baseGeom = windowPtr -> geometry();
        }
        else
        {
            nextGeom.setX(baseGeom.x() + 40*camCnt);
            nextGeom.setY(baseGeom.y() + 40*camCnt);
            nextGeom.setWidth(baseGeom.width());
            nextGeom.setHeight(baseGeom.height());
            windowPtr -> setGeometry(nextGeom);
        }
        windowPtrList.push_back(windowPtr);
    }
    return app.exec();
}

