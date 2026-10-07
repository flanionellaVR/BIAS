#include "video_utils.hpp"
#include <opencv2/videoio.hpp>
#include <iostream>
#include <sstream>
#include "exception.hpp"
//#include "stampedImage.hpp"


namespace bias {


    videoBackend::videoBackend() {
        isOpen_ = false;
        dt_ = 1.0 / 30.0;
    }

    videoBackend::videoBackend(QString file) {

        filename = file;
        cap_.open(filename.toStdString().c_str());
        isOpen_ = cap_.isOpened();
        dt_ = 1.0 / 30.0;
        if (isOpen_) {
            float fps = getFPS();
            if (fps > 0.0f) { dt_ = 1.0 / (double)fps; }
        }

    }


    void videoBackend::releaseCapObject() {

        cap_.release();
        isOpen_ = false;

    }

    void videoBackend::checkCapOpen() {

        if (!isOpen_) {
            std::stringstream ssError;
            ssError << "video capture object not open: " << filename.toStdString();
            throw RuntimeError(ERROR_VIDEO_OPEN, ssError.str());
		}

	}
    
    cv::Mat videoBackend::grabImage() {

        cv::Mat frame;
        cv::Mat grey;

        checkCapOpen();
        
        cap_.read(frame);

        if (frame.empty())
            return frame;

        // convert the frame to grayscale
        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        return grey; 

    }

    void videoBackend::convertImagetoFloat(cv::Mat& img) {

        // convert the frame into float32
        img.convertTo(img, CV_32FC1);
        img = img / 255;

    }


    int videoBackend::getImageHeight() {
        checkCapOpen();
        return cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    }

    int videoBackend::getImageWidth() {
        checkCapOpen();
        return cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    }


    float videoBackend::getFPS() {
        checkCapOpen();
        return cap_.get(cv::CAP_PROP_FPS);
    }


    int videoBackend::getNumFrames() {

        return cap_.get(cv::CAP_PROP_FRAME_COUNT);

    }

    int videoBackend::getCurrentFrameNumber() {

        return cap_.get(cv::CAP_PROP_POS_FRAMES);
    }

    bool videoBackend::setFrame(int f) {

		return cap_.set(cv::CAP_PROP_POS_FRAMES, f);

	}

    TimeStamp videoBackend::getImageTimeStamp() {

        TimeStamp timestamp = TimeStamp();
        int fr = getCurrentFrameNumber();
        double t = fr * dt_;
        timestamp.seconds = (int)t;
        timestamp.microSeconds = (int) ((t - timestamp.seconds) * 1e6);
        return timestamp;

	}

    void videoBackend::setBufferSize() {

        cap_.set(cv::CAP_PROP_BUFFERSIZE, 3);

    }

    std::string videoBackend::type2str(int type) {

        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
        }

        r += "C";
        r += (chans + '0');

        return r;
    }

    void videoBackend::readVidFrames(std::vector<cv::Mat>& vid_frames) {
        
        checkCapOpen();
        int num_frames = getNumFrames();
        int height = getImageHeight();
        int width = getImageWidth();

        vid_frames.resize(num_frames, cv::Mat());

        for (int frm_id = 0; frm_id < num_frames; frm_id++)
        {
            vid_frames[frm_id] = grabImage();
        }
        std::cout << "Finished reading movie" << std::endl;
    }

    void videoBackend::preprocess_vidFrame(cv::Mat& cur_Img)
    {
        cv::Mat greyImg;
        convertImagetoFloat(cur_Img);
        
    }

}