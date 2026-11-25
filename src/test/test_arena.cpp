// Test program for the Arena SDK (LUCID Vision Labs) backend.
//
// Exercises the backend through the same camera facade the GUI uses and prints
// PASS/FAIL for each check. Settings changed by the tests are restored at the
// end. No GUI/highgui functions are used so that it also runs on machines
// with a headless OpenCV.
//
// Usage:
//
//   test_arena                run all tests on the first camera found
//   test_arena --nodes        also dump the camera's node map
//   test_arena --save <dir>   also save a few grabbed frames as PNG files to <dir>
//   test_arena --info         only enumerate + print camera information
//
#include <cmath>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "camera_facade.hpp"
#include "camera_device_arena.hpp"

using namespace bias;

static int numPass = 0;
static int numFail = 0;

static void check(bool ok, const std::string &what)
{
    std::cout << (ok ? "  PASS  " : "  FAIL  ") << what << std::endl;
    if (ok) { numPass++; } else { numFail++; }
}

struct GrabStats
{
    int numFrames = 0;
    int numEmpty = 0;
    int rows = 0;
    int cols = 0;
    int type = -1;
    double meanValue = 0.0;
    double meanDt = 0.0;     // mean time between frames, camera clock (s)
    double minDt = 0.0;
    double maxDt = 0.0;
    double wallFps = 0.0;    // frames per second measured on the PC clock
    bool timeStampsIncreasing = true;
    cv::Mat lastImage;
};

static double toSeconds(TimeStamp ts)
{
    return double(ts.seconds) + 1.0e-6*double(ts.microSeconds);
}

// Grabs numFrames frames (or gives up after timeoutSec)
static GrabStats grabFrames(Camera &camera, int numFrames, double timeoutSec = 20.0)
{
    GrabStats stats;
    double sumMean = 0.0;
    double sumDt = 0.0;
    double lastT = 0.0;
    int numDt = 0;

    auto t0 = std::chrono::steady_clock::now();
    auto tFirst = t0;
    auto tLast = t0;

    while (stats.numFrames < numFrames)
    {
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (elapsed > timeoutSec)
        {
            break;
        }

        cv::Mat image = camera.grabImage();
        if (image.empty())
        {
            stats.numEmpty++;
            continue;
        }

        auto tNow = std::chrono::steady_clock::now();
        if (stats.numFrames == 0) { tFirst = tNow; }
        tLast = tNow;

        double t = toSeconds(camera.getImageTimeStamp());
        if (stats.numFrames > 0)
        {
            double dt = t - lastT;
            if (dt <= 0.0) { stats.timeStampsIncreasing = false; }
            sumDt += dt;
            if ((numDt == 0) || (dt < stats.minDt)) { stats.minDt = dt; }
            if ((numDt == 0) || (dt > stats.maxDt)) { stats.maxDt = dt; }
            numDt++;
        }
        lastT = t;

        stats.rows = image.rows;
        stats.cols = image.cols;
        stats.type = image.type();
        sumMean += cv::mean(image)[0];
        stats.numFrames++;
        stats.lastImage = image;
    }

    if (stats.numFrames > 0) { stats.meanValue = sumMean/stats.numFrames; }
    if (numDt > 0) { stats.meanDt = sumDt/numDt; }
    double wallSpan = std::chrono::duration<double>(tLast - tFirst).count();
    if ((stats.numFrames > 1) && (wallSpan > 0.0)) { stats.wallFps = double(stats.numFrames - 1)/wallSpan; }
    return stats;
}

static void printStats(const std::string &label, const GrabStats &s)
{
    std::cout << "        " << label << ": " << s.numFrames << " frames, " << s.cols << "x" << s.rows;
    std::cout << ", cv type " << s.type << ", mean " << s.meanValue;
    std::cout << ", camera dt mean/min/max " << 1000.0*s.meanDt << "/" << 1000.0*s.minDt << "/" << 1000.0*s.maxDt << " ms";
    std::cout << " (" << ((s.meanDt > 0.0) ? 1.0/s.meanDt : 0.0) << " fps)";
    std::cout << ", PC clock " << s.wallFps << " fps, empty grabs " << s.numEmpty << std::endl;
}

static void setAbsolute(Camera &camera, PropertyType propType, float value)
{
    Property prop = camera.getProperty(propType);
    prop.absoluteControl = true;
    prop.absoluteValue = value;
    prop.autoActive = false;
    prop.onePush = false;
    camera.setProperty(prop);
}


int main(int argc, char *argv[])
{
    bool dumpNodes = false;
    bool infoOnly = false;
    std::string saveDir;
    for (int i=1; i<argc; i++)
    {
        std::string arg(argv[i]);
        if (arg == "--nodes") { dumpNodes = true; }
        else if (arg == "--info") { infoOnly = true; }
        else if ((arg == "--save") && (i+1 < argc)) { saveDir = std::string(argv[++i]); }
        else
        {
            std::cout << "usage: test_arena [--info] [--nodes] [--save <dir>]" << std::endl;
            return 2;
        }
    }

    try
    {
        // Enumeration
        // --------------------------------------------------------------------
        std::cout << std::endl << "Enumeration" << std::endl;
        CameraFinder camFinder;
        std::cout << "        " << camFinder.getCameraSdkVersionString() << std::endl;
        GuidList guidList = camFinder.getGuidList();
        check(!guidList.empty(), "at least one camera found");
        for (auto guid : guidList)
        {
            std::cout << "        guid: " << guid.toString() << ", camera lib: " << guid.getCameraLib() << std::endl;
        }
        if (guidList.empty())
        {
            return 1;
        }

        Guid guid = guidList.front();
        check(guid.getCameraLib() == CAMERA_LIB_ARENA, "guid belongs to the Arena camera library");

        // Node map dump - uses the device class directly
        // --------------------------------------------------------------------
        if (dumpNodes)
        {
            CameraDevice_arena camDev(guid);
            camDev.connect();
            std::cout << std::endl << "Node map" << std::endl;
            std::cout << camDev.nodeMapToString() << std::endl;
            camDev.disconnect();
        }

        // Connect
        // --------------------------------------------------------------------
        std::cout << std::endl << "Connect" << std::endl;
        Camera camera(guid);
        camera.connect();
        check(camera.isConnected(), "connected");
        std::cout << "        vendor: " << camera.getVendorName() << ", model: " << camera.getModelName();
        std::cout << ", color: " << camera.isColor() << std::endl;
        check(!camera.getVendorName().empty() && !camera.getModelName().empty(), "vendor and model name available");
        check(camera.getVideoMode() == VIDEOMODE_FORMAT7, "video mode is Format7");
        check(camera.getTriggerType() == TRIGGER_INTERNAL, "trigger is internal after connect");

        Format7Settings origSettings = camera.getFormat7Settings();
        Format7Info format7Info = camera.getFormat7Info(origSettings.mode);
        std::cout << std::endl << "Format7 settings" << std::endl;
        origSettings.print();
        std::cout << std::endl << "Format7 info" << std::endl;
        format7Info.print();

        PixelFormatList pixelFormatList = camera.getListOfSupportedPixelFormats(origSettings.mode);
        std::cout << std::endl << "        supported pixel formats:";
        for (auto format : pixelFormatList)
        {
            std::cout << " " << getPixelFormatString(format);
        }
        std::cout << std::endl;
        check(!pixelFormatList.empty(), "camera supports at least one pixel format known to BIAS");
        check(camera.validateFormat7Settings(origSettings), "current Format7 settings validate");

        std::cout << std::endl << "Properties" << std::endl;
        camera.printAllProperties();
        camera.printAllPropertyInfos();

        if (infoOnly)
        {
            camera.disconnect();
            std::cout << std::endl << "passed " << numPass << ", failed " << numFail << std::endl;
            return (numFail == 0) ? 0 : 1;
        }

        // Remember settings to restore
        Property origShutter = camera.getProperty(PROPERTY_TYPE_SHUTTER);
        Property origGain = camera.getProperty(PROPERTY_TYPE_GAIN);
        Property origFrameRate = camera.getProperty(PROPERTY_TYPE_FRAME_RATE);
        Property origBrightness = camera.getProperty(PROPERTY_TYPE_BRIGHTNESS);

        // Capture at the current settings
        // --------------------------------------------------------------------
        std::cout << std::endl << "Capture (current settings)" << std::endl;
        camera.startCapture();
        check(camera.isCapturing(), "capturing");
        GrabStats full = grabFrames(camera, 50);
        printStats("full", full);
        check(full.numFrames == 50, "grabbed 50 frames");
        check((full.cols == int(origSettings.width)) && (full.rows == int(origSettings.height)), "image size matches Format7 settings");
        check(full.timeStampsIncreasing, "timestamps strictly increasing");
        PropertyInfo frameRateInfo = camera.getPropertyInfo(PROPERTY_TYPE_FRAME_RATE);
        if (frameRateInfo.present && (full.meanDt > 0.0))
        {
            double camFps = camera.getProperty(PROPERTY_TYPE_FRAME_RATE).absoluteValue;
            std::cout << "        camera frame rate setting: " << camFps << " fps" << std::endl;
            check(std::fabs(1.0/full.meanDt - camFps) < 0.02*camFps, "timestamp interval matches the camera's frame rate (2%)");
        }
        if (!saveDir.empty() && !full.lastImage.empty())
        {
            // Save a reduced copy - full frames can be very large
            cv::Mat small = full.lastImage;
            std::string fileName = saveDir + "/test_arena_full.png";
            check(cv::imwrite(fileName, small), "saved " + fileName);
        }
        camera.stopCapture();
        check(!camera.isCapturing(), "capture stopped");

        // Shutter (exposure) and gain
        // --------------------------------------------------------------------
        std::cout << std::endl << "Shutter and gain" << std::endl;
        PropertyInfo shutterInfo = camera.getPropertyInfo(PROPERTY_TYPE_SHUTTER);
        PropertyInfo gainInfo = camera.getPropertyInfo(PROPERTY_TYPE_GAIN);
        check(shutterInfo.present, "shutter property present");
        check(gainInfo.present, "gain property present");
        if (shutterInfo.present && gainInfo.present)
        {
            float shortExposure = std::max(shutterInfo.minAbsoluteValue, 500.0f);
            float longExposure = std::min(shutterInfo.maxAbsoluteValue, 4.0f*shortExposure);

            setAbsolute(camera, PROPERTY_TYPE_GAIN, gainInfo.minAbsoluteValue);
            setAbsolute(camera, PROPERTY_TYPE_SHUTTER, shortExposure);
            float readShort = camera.getProperty(PROPERTY_TYPE_SHUTTER).absoluteValue;
            std::cout << "        shutter set " << shortExposure << ", read back " << readShort << std::endl;
            check(std::fabs(readShort - shortExposure) < 0.05f*shortExposure, "short shutter value read back (5%)");
            check(!camera.getProperty(PROPERTY_TYPE_SHUTTER).autoActive, "auto exposure is off after setting the shutter");

            camera.startCapture();
            grabFrames(camera, 5);  // let the new settings take effect
            GrabStats shortStats = grabFrames(camera, 10);
            camera.stopCapture();
            printStats("short exposure", shortStats);

            setAbsolute(camera, PROPERTY_TYPE_SHUTTER, longExposure);
            float readLong = camera.getProperty(PROPERTY_TYPE_SHUTTER).absoluteValue;
            std::cout << "        shutter set " << longExposure << ", read back " << readLong << std::endl;
            check(std::fabs(readLong - longExposure) < 0.05f*longExposure, "long shutter value read back (5%)");

            camera.startCapture();
            grabFrames(camera, 5);
            GrabStats longStats = grabFrames(camera, 10);
            camera.stopCapture();
            printStats("long exposure", longStats);
            check(longStats.meanValue > shortStats.meanValue, "longer exposure gives a brighter image");

            // Shutter via the integer (slider) value used by the GUI
            Property shutterProp = camera.getProperty(PROPERTY_TYPE_SHUTTER);
            shutterProp.absoluteControl = false;
            shutterProp.value = (shutterInfo.minValue + shutterInfo.maxValue)/2;
            unsigned int requestedValue = shutterProp.value;
            camera.setProperty(shutterProp);
            unsigned int readValue = camera.getProperty(PROPERTY_TYPE_SHUTTER).value;
            std::cout << "        shutter slider set " << requestedValue << ", read back " << readValue << std::endl;
            check((readValue + 1 >= requestedValue) && (readValue <= requestedValue + 1), "shutter slider value read back (+/-1)");

            // Gain
            setAbsolute(camera, PROPERTY_TYPE_SHUTTER, longExposure);
            float highGain = std::min(gainInfo.maxAbsoluteValue, gainInfo.minAbsoluteValue + 12.0f);
            setAbsolute(camera, PROPERTY_TYPE_GAIN, highGain);
            float readGain = camera.getProperty(PROPERTY_TYPE_GAIN).absoluteValue;
            std::cout << "        gain set " << highGain << ", read back " << readGain << std::endl;
            check(std::fabs(readGain - highGain) < 0.5f, "gain value read back");

            camera.startCapture();
            grabFrames(camera, 5);
            GrabStats gainStats = grabFrames(camera, 10);
            camera.stopCapture();
            printStats("high gain", gainStats);
            check(gainStats.meanValue > longStats.meanValue, "higher gain gives a brighter image");
            if (!saveDir.empty() && !gainStats.lastImage.empty())
            {
                std::string fileName = saveDir + "/test_arena_gain.png";
                check(cv::imwrite(fileName, gainStats.lastImage), "saved " + fileName);
            }
            setAbsolute(camera, PROPERTY_TYPE_GAIN, gainInfo.minAbsoluteValue);
        }

        // Region of interest
        // --------------------------------------------------------------------
        std::cout << std::endl << "Region of interest (Format7)" << std::endl;
        {
            Format7Settings roi = origSettings;
            roi.width  = ((format7Info.maxWidth/4)/format7Info.imageHStepSize)*format7Info.imageHStepSize;
            roi.height = ((format7Info.maxHeight/4)/format7Info.imageVStepSize)*format7Info.imageVStepSize;
            roi.offsetX = (((format7Info.maxWidth - roi.width)/2)/format7Info.offsetHStepSize)*format7Info.offsetHStepSize;
            roi.offsetY = (((format7Info.maxHeight - roi.height)/2)/format7Info.offsetVStepSize)*format7Info.offsetVStepSize;
            check(camera.validateFormat7Settings(roi), "centered quarter-size ROI validates");

            Format7Settings bad = roi;
            bad.width = format7Info.maxWidth + format7Info.imageHStepSize;
            check(!camera.validateFormat7Settings(bad), "oversized ROI is rejected");

            camera.setFormat7Configuration(roi, 100.0f);
            Format7Settings readRoi = camera.getFormat7Settings();
            std::cout << "        ROI set " << roi.width << "x" << roi.height << "+" << roi.offsetX << "+" << roi.offsetY;
            std::cout << ", read back " << readRoi.width << "x" << readRoi.height << "+" << readRoi.offsetX << "+" << readRoi.offsetY << std::endl;
            check((readRoi.width == roi.width) && (readRoi.height == roi.height) &&
                  (readRoi.offsetX == roi.offsetX) && (readRoi.offsetY == roi.offsetY), "ROI read back");

            // Frame rate - a smaller ROI allows a higher rate
            PropertyInfo roiFrameRateInfo = camera.getPropertyInfo(PROPERTY_TYPE_FRAME_RATE);
            std::cout << "        frame rate range with ROI: " << roiFrameRateInfo.minAbsoluteValue << " - " << roiFrameRateInfo.maxAbsoluteValue << " fps" << std::endl;

            camera.startCapture();
            GrabStats roiStats = grabFrames(camera, 100);
            camera.stopCapture();
            printStats("ROI", roiStats);
            check(roiStats.numFrames == 100, "grabbed 100 ROI frames");
            check((roiStats.cols == int(roi.width)) && (roiStats.rows == int(roi.height)), "ROI image size matches");
            if (!saveDir.empty() && !roiStats.lastImage.empty())
            {
                std::string fileName = saveDir + "/test_arena_roi.png";
                check(cv::imwrite(fileName, roiStats.lastImage), "saved " + fileName);
            }

            // Frame rate
            if (roiFrameRateInfo.present)
            {
                float targetFps = std::max(roiFrameRateInfo.minAbsoluteValue, std::min(roiFrameRateInfo.maxAbsoluteValue, 25.0f));
                setAbsolute(camera, PROPERTY_TYPE_FRAME_RATE, targetFps);
                float readFps = camera.getProperty(PROPERTY_TYPE_FRAME_RATE).absoluteValue;
                std::cout << "        frame rate set " << targetFps << ", read back " << readFps << std::endl;
                check(std::fabs(readFps - targetFps) < 0.02f*targetFps, "frame rate read back (2%)");

                camera.startCapture();
                grabFrames(camera, 5);
                GrabStats fpsStats = grabFrames(camera, 50);
                camera.stopCapture();
                printStats("frame rate", fpsStats);
                check((fpsStats.meanDt > 0.0) && (std::fabs(1.0/fpsStats.meanDt - targetFps) < 0.02*targetFps),
                        "measured frame rate matches setting (2%, camera clock)");
                check(std::fabs(fpsStats.wallFps - targetFps) < 0.1*targetFps, "measured frame rate matches setting (10%, PC clock)");
            }

            // Pixel formats
            for (auto format : pixelFormatList)
            {
                Format7Settings fmtSettings = roi;
                fmtSettings.pixelFormat = format;
                std::string formatName = getPixelFormatString(format);
                if (!camera.validateFormat7Settings(fmtSettings))
                {
                    check(false, "pixel format " + formatName + " validates");
                    continue;
                }
                camera.setFormat7Configuration(fmtSettings, 100.0f);
                check(camera.getFormat7Settings().pixelFormat == format, "pixel format " + formatName + " set");
                camera.startCapture();
                GrabStats fmtStats = grabFrames(camera, 10);
                camera.stopCapture();
                printStats(formatName, fmtStats);
                check(fmtStats.numFrames == 10, "grabbed 10 frames as " + formatName);
            }
        }

        // Trigger
        // --------------------------------------------------------------------
        std::cout << std::endl << "Trigger" << std::endl;
        {
            camera.setTriggerExternal();
            check(camera.getTriggerType() == TRIGGER_EXTERNAL, "trigger type is external after setTriggerExternal");
            camera.startCapture();
            GrabStats trigStats = grabFrames(camera, 1, 1.0);
            camera.stopCapture();
            std::cout << "        frames in 1 s with external trigger and no trigger signal: " << trigStats.numFrames;
            std::cout << " (empty grabs " << trigStats.numEmpty << ")" << std::endl;
            check(trigStats.numFrames == 0, "no frames without a trigger signal, grabImage returns empty images");
            camera.setTriggerInternal();
            check(camera.getTriggerType() == TRIGGER_INTERNAL, "trigger type is internal after setTriggerInternal");
            camera.startCapture();
            GrabStats freeStats = grabFrames(camera, 10);
            camera.stopCapture();
            check(freeStats.numFrames == 10, "free running again after setTriggerInternal");
        }

        // Restore
        // --------------------------------------------------------------------
        std::cout << std::endl << "Restore settings" << std::endl;
        camera.setFormat7Configuration(origSettings, 100.0f);
        Format7Settings restored = camera.getFormat7Settings();
        check((restored.width == origSettings.width) && (restored.height == origSettings.height) &&
              (restored.offsetX == origSettings.offsetX) && (restored.offsetY == origSettings.offsetY) &&
              (restored.pixelFormat == origSettings.pixelFormat), "Format7 settings restored");
        if (origFrameRate.present) { setAbsolute(camera, PROPERTY_TYPE_FRAME_RATE, origFrameRate.absoluteValue); }
        if (origBrightness.present) { setAbsolute(camera, PROPERTY_TYPE_BRIGHTNESS, origBrightness.absoluteValue); }
        if (origGain.present) { setAbsolute(camera, PROPERTY_TYPE_GAIN, origGain.absoluteValue); }
        if (origShutter.present) { setAbsolute(camera, PROPERTY_TYPE_SHUTTER, origShutter.absoluteValue); }
        std::cout << "        shutter " << camera.getProperty(PROPERTY_TYPE_SHUTTER).absoluteValue;
        std::cout << ", gain " << camera.getProperty(PROPERTY_TYPE_GAIN).absoluteValue;
        std::cout << ", frame rate " << camera.getProperty(PROPERTY_TYPE_FRAME_RATE).absoluteValue << std::endl;

        // Reconnect - the camera must be usable again after a disconnect
        // --------------------------------------------------------------------
        std::cout << std::endl << "Reconnect" << std::endl;
        camera.disconnect();
        check(!camera.isConnected(), "disconnected");
        camera.connect();
        check(camera.isConnected(), "connected again");
        camera.startCapture();
        GrabStats again = grabFrames(camera, 5);
        camera.stopCapture();
        check(again.numFrames == 5, "grabbed frames after reconnect");
        camera.disconnect();
    }
    catch (RuntimeError &runtimeError)
    {
        std::cout << std::endl << "RuntimeError: id = " << runtimeError.id() << ", what = " << runtimeError.what() << std::endl;
        numFail++;
    }
    catch (std::exception &e)
    {
        std::cout << std::endl << "exception: " << e.what() << std::endl;
        numFail++;
    }

    std::cout << std::endl << "passed " << numPass << ", failed " << numFail << std::endl;
    return (numFail == 0) ? 0 : 1;
}
