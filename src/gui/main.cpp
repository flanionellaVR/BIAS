#include <list>
#include <QApplication>
#include <QSharedPointer>
#include <QMessageBox>
#include "camera_window.hpp"
#include "camera_facade.hpp"
#include "affinity.hpp"
#include <iostream>
#include <QCommandLineParser>


// ------------------------------------------------------------------------
// TO DO ... temporary main function. Currently just opens a camera
// window for each camera found attached to the system.
// ------------------------------------------------------------------------
int main (int argc, char *argv[])
{
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

    // Query the Spinnaker library version once for the About dialog; not fatal if it fails.
    try
    {
        bias::CameraWindow::spinnakerVersionString =
            QString::fromStdString(cameraFinder.getSpinnakerVersionString());
    }
    catch (bias::RuntimeError &runtimeError)
    {
        std::cerr << "Unable to get Spinnaker library version: " << runtimeError.what() << std::endl;
    }

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

