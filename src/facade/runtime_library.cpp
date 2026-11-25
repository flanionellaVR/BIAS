#include "runtime_library.hpp"
#include <map>
#include <mutex>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace bias
{

    bool isRuntimeLibraryAvailable(const std::string &libraryFileName)
    {
#ifdef _WIN32
        // Results are cached: a failed search is not repeated on every call
        static std::mutex cacheMutex;
        static std::map<std::string, bool> cache;

        std::lock_guard<std::mutex> lock(cacheMutex);
        auto it = cache.find(libraryFileName);
        if (it != cache.end())
        {
            return it -> second;
        }

        // Don't show the "DLL was not found" system dialog for a missing
        // library or a missing dependency of it - failing is an expected
        // outcome here.
        UINT oldErrorMode = SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX);
        HMODULE hModule = LoadLibraryA(libraryFileName.c_str());
        SetErrorMode(oldErrorMode);

        // Note: the library is deliberately not released again - it is about
        // to be used.
        bool available = (hModule != NULL);
        cache[libraryFileName] = available;
        return available;
#else
        (void)libraryFileName;
        return true;
#endif
    }

} // namespace bias
