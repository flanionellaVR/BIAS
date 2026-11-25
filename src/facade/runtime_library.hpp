#ifndef BIAS_RUNTIME_LIBRARY_HPP
#define BIAS_RUNTIME_LIBRARY_HPP

#include <string>

namespace bias
{
    // Returns true if the shared library with the given file name (e.g.
    // "SpinnakerC_v140.dll") can be loaded, together with everything it
    // depends on, from the usual search locations (directory of the
    // executable, system directories, PATH).
    //
    // The camera SDK libraries are linked "delay loaded" on Windows: they are
    // not needed to start the program and are only loaded when one of their
    // functions is called for the first time - which ends the program if the
    // library is missing. This function is the check to make before the first
    // call into a camera SDK, so that BIAS can run on a machine where that SDK
    // is not installed.
    //
    // A library found by this function stays loaded.
    //
    // On other platforms the libraries are needed at program start anyway, so
    // this always returns true.
    bool isRuntimeLibraryAvailable(const std::string &libraryFileName);
}

#endif // #ifndef BIAS_RUNTIME_LIBRARY_HPP
