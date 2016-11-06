////////////////////////////////////////////////////////////////////////////////////
///
///  \file capture.h
///  \brief This file contains an interface for taking screen shots.  It can
///  also spawn a thread for continuous capture and logging of screen data.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 14 January 2009
///  <br>Copyright (c) 2009
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dbarber@ist.ucf.edu
///  <br>Web:  http://active.ist.ucf.edu
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ACTIVE LAB, IST, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
///
///  THIS SOFTWARE IS PROVIDED BY THE ACTIVE LAB''AS IS'' AND ANY
///  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
///  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
///  DISCLAIMED. IN NO EVENT SHALL UCF BE LIABLE FOR ANY
///  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
///  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
///  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
///  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
///  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
///  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
////////////////////////////////////////////////////////////////////////////////////
#ifndef __SCREEN_CAPTURE__H
#define __SCREEN_CAPTURE__H

#include "cxutils/Thread.h"
#include "cxutils/images/Image.h"
#include <vector>

class TiXmlNode;

namespace CxUtils
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Capture
    ///   \brief Interface for capturing and logging screen information.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class CX_UTILS_DLL ScreenCapture
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \class Screenshot
        ///   \brief Simple class to storing screen shot data.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        class CX_UTILS_DLL Screenshot : public CxUtils::Image
        {
        public:
            typedef std::vector<Screenshot*> List;
            Screenshot();
            Screenshot(const unsigned char* image, 
                       const unsigned int width, 
                       const unsigned int height);
            Screenshot(const Screenshot& ss);
            ~Screenshot();
            bool LoadScreenshot(const std::string& filename);
            static bool ExtractInfoFromFilename(const std::string& filename, 
                                                CxUtils::Time& time, 
                                                unsigned int& participantID);
            Screenshot& operator=(const Screenshot& ss);
            unsigned int mParticipantID;///<  Participant ID.
            CxUtils::Time mTimeStamp;   ///<  Time Stamp of data.
        };
        ScreenCapture();
        ~ScreenCapture();
        bool CaptureScreen(int screen = -1);
        bool SaveImage(const std::string& filename) const;
        bool SetScaleFactor(const double scale);
        unsigned int GetWidth() const { return mWidth; }
        unsigned int GetHeight() const { return mHeight; }
        unsigned char* GetImageData() const { return mpImageData; }
        Screenshot GetScreenshot();
        void SetCaptureDelayMs(const unsigned int delay) { mCaptureDelayMs = delay; }
        static bool GetScreenResolution(unsigned int& width, unsigned int& height, int screen = -1);
        bool StartCaptureThread(const std::string& directory, unsigned int id = 0, const double timeLimit = 0.0);
        bool StartCaptureThreadFromXML(const std::string& xmlFile, const unsigned int id = 0);
        bool StartCaptureThreadFromXMLElement(TiXmlNode* element, const unsigned int id = 0);
        void StopCaptureThread();
        void SetScreenNumber(int num){mScreenNumber = num;}
        unsigned int GetCaptureCount() const { return mCaptureCount; }
        bool IsActive() const;
    protected:        
        static void CaptureThread(void* arg);
        static void WriterThread(void* arg);
        CxUtils::Thread mCaptureThread;     ///<  Thread to capture screen shots.
        CxUtils::Thread mWriterThread;      ///<  Thread to write images to disk.
        volatile unsigned int mCaptureDelayMs;  ///<  Delay time between frame grabs.
        Screenshot::List mScreenshots;      ///<  Queue of capture screenshots to save to disk.
        CxUtils::Mutex mScreenshotsMutex;   ///<  Mutex for thread protection of data.
        std::string mDirectory;             ///<  Directory to save data to during continuous capture.
        unsigned int mID;                   ///<  ID number to use for generating file names.
        double mTimeLimitSeconds;           ///<  How long to log data for (seconds).
        volatile unsigned int mCaptureCount;///<  Number of frames captured.
        volatile bool mCaptureQuitFlag;     ///<  Indicates continuous capture is complete.
        double mScaleFactor;                ///<  How much to scale an image.
        unsigned int mWidth;        ///<  Screen width in pixels.
        unsigned int mHeight;       ///<  Screen height in pixels.
        int mScreenNumber;          ///<  Screen Number, -1 for all screens, 0, 1, etc to specificy specific screen
        unsigned char* mpImageData; ///<  Buffer containing image data in captured from screen.
        unsigned char* mpCompressionBuffer;             ///<  Buffer for storing compressed image data.
        unsigned int mCompressionBufferSize;            ///<  Size of compression buffer.
        CxUtils::JPEG::Compressor    mJPEGCompressor;   ///<  For compressing JPEG data.
#ifdef WIN32
        char* mpBuffer;             ///<  Pointer to Windows buffer.
#else
        void* mpDisplay;            ///<  Display pointer.
#endif
    };
}


#endif
/*  End of File */
