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
#include "cxutils/ScreenCapture.h"
#include <cxutils/Timer.h>

#include <string.h>
#include <cxutils/FileIO.h>
#include <tinyxml.h>

#ifdef WIN32
#include <windows.h>
#else
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif

using namespace CxUtils;

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot::Screenshot()
{
    mParticipantID = 0;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
///   \param[in] image Pointer to image data to copy.  All image data is in
///              RGB pixel format.
///   \param[in] width Image width in pixels.
///   \param[in] height Image height in pixels.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot::Screenshot(const unsigned char* image,
                                const unsigned int width,
                                const unsigned int height)
{
    if(width > 0 && height > 0)
    {
        this->Create((unsigned short)width, (unsigned short)height, 3, image);
        mTimeStamp.SetCurrentTime();
    }
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Copy constructor.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot::Screenshot(const ScreenCapture::Screenshot& ss)
{
    *this = ss;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot::~Screenshot()
{
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Extracts participant and time stamp data from the file name of a
///   saved screen shot.
///
///   \param[in] filename Name of a saved screen shot.
///   \param[in] time Time of when screen shot was taken (from file name).
///   \param[in] participantID ID of the participant (from file name).
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::Screenshot::ExtractInfoFromFilename(const std::string& filename,
                                                  CxUtils::Time& time,
                                                  unsigned int& participantID)
{
    //double timeSeconds;

    time.mDay = 0;

    const char * str = filename.c_str();
    int lastSlash = -1;
    for (unsigned int i = 0; i < filename.size(); i++)
    {
        if (str[i] == '\\' || str[i] == '/')
            lastSlash = i;
    }

    std::string justFileName = filename;
    if (lastSlash != -1)
        justFileName = filename.substr(lastSlash+1, filename.length()-lastSlash-5);

    int h, m, s, ms;

    if(sscanf(justFileName.c_str(),
              "%d - %d.%d.%d.%d",
              &participantID,
              &h,
              &m,
              &s,
              &ms/*,
              &timeSeconds*/) == 5)
    {

      time.mHour = h;
      time.mMinute = m;
      time.mSecond = s;
      time.mMilliseconds = ms;

        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Loads JPEG Screenshot from disk.
///
///   \param[in] filename Filename of screenshot to load.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::Screenshot::LoadScreenshot(const std::string& filename)
{
    return this->Load(filename) > 0 ? true : false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets equal to.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot& ScreenCapture::Screenshot::operator=(const ScreenCapture::Screenshot& ss)
{
    *((CxUtils::Image*)(this)) = *((CxUtils::Image*)(&ss));
    mTimeStamp = ss.mTimeStamp;
    mParticipantID = ss.mParticipantID;

    return *this;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Constructor.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::ScreenCapture() : mID(0),
                     mCaptureCount(0),
                     mCaptureQuitFlag(false),
                     mScaleFactor(1.0),
                     mWidth(0),
                     mHeight(0),
                     mpImageData(0),
                     mpCompressionBuffer(0),
                     mCompressionBufferSize(0),
                     mScreenNumber(-1)
{
    mCaptureDelayMs = 25;
#ifdef WIN32
    mpBuffer = NULL;
#else
    mpDisplay = NULL;
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Destructor.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::~ScreenCapture()
{
    StopCaptureThread();

    for(unsigned int i = 0; i < 10000; i++)
    {
        if(mWriterThread.IsThreadActive() == false)
            break;
        else
            CxUtils::SleepMs(1);
    }
    mWriterThread.StopThread(1);

    Screenshot::List::iterator image;
    for(image = mScreenshots.begin(); image != mScreenshots.end(); image++)
    {
        delete *image;
    }
    mScreenshots.clear();

    if(mpImageData)
    {
        delete[] mpImageData;
    }
    mpImageData = NULL;

    if(mpCompressionBuffer)
    {
        delete[] mpCompressionBuffer;
    }
    mpCompressionBuffer = NULL;
    mCompressionBufferSize = 0;
#ifdef WIN32
    if(mpBuffer)
    {
        delete[] mpBuffer;
    }
    mpBuffer = NULL;
#else
    if(mpDisplay)
    {
        XCloseDisplay((Display*)mpDisplay);
    }
    mpDisplay = NULL;
#endif

}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Captures the current desktop (full screen) and saves to internal
///          data members.
///   \param[in] screen Identify screen to capture in the case of multiple monitors.
///           -1 by default, will capture entire virtual screen.
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::CaptureScreen(int screen)
{
    bool result = false;
#ifdef WIN32
    BITMAPINFO  *bitmapInfo;

    HWND windowHandle = GetDesktopWindow();
    HDC hdcSrc = GetWindowDC(windowHandle);
    HDC hdcDst = CreateCompatibleDC(hdcSrc);

    if(mpBuffer == NULL)
    {
        GetScreenResolution(mWidth, mHeight,screen);
        mpBuffer = new char[mWidth*mHeight*4 + sizeof(BITMAPINFOHEADER)];
    }
    if(mpImageData == NULL)
    {
        GetScreenResolution(mWidth, mHeight,screen);
        mpImageData = new unsigned char[mWidth*mHeight*3];
    }
    bitmapInfo = (BITMAPINFO *)mpBuffer;
    ZeroMemory(bitmapInfo, sizeof(BITMAPINFO));
    HBITMAP bitmap = CreateCompatibleBitmap(hdcSrc, (int)mWidth, (int)mHeight);
    SelectObject(hdcDst, bitmap);

    if(BitBlt(hdcDst, 0, 0, (int)mWidth, (int)mHeight, hdcSrc, 0, 0, SRCCOPY) != 0)
    {

        bitmapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);

        // Call with NULL output buffer so that
        // the header information for the bitmap will be calculated.
        if (GetDIBits(hdcDst,
            bitmap,
            0,
            1,
            NULL,
            bitmapInfo,
            DIB_RGB_COLORS) == 0)
        {
            return result;
        }

        // Read the scanlines from DC to buffer.
        if (GetDIBits(hdcDst,
            bitmap,
            0,
            bitmapInfo->bmiHeader.biHeight,
            mpBuffer + sizeof(BITMAPINFOHEADER),
            bitmapInfo,
            DIB_RGB_COLORS) == 0)
        {
            return result;
        }

        unsigned char pixelSize = bitmapInfo->bmiHeader.biBitCount/8;
        // Extract RGB data and flip image.
        for (unsigned int i= 0; i < mHeight; i++)
        {
            for (unsigned int j = 0; j < mWidth; j++)
            {
                mpImageData[i*mWidth*3+j*3 + 0] = mpBuffer[(mHeight-i-1)*mWidth*pixelSize +j*pixelSize+ 0 + bitmapInfo->bmiHeader.biSize];
                mpImageData[i*mWidth*3+j*3 + 1] = mpBuffer[(mHeight-i-1)*mWidth*pixelSize +j*pixelSize+ 1 + bitmapInfo->bmiHeader.biSize];
                mpImageData[i*mWidth*3+j*3 + 2] = mpBuffer[(mHeight-i-1)*mWidth*pixelSize +j*pixelSize+ 2 + bitmapInfo->bmiHeader.biSize];
            }
        }

        DeleteObject(bitmap);
        DeleteDC(hdcDst);
        ReleaseDC(0, hdcSrc);

        result = true;
    }
#else
    Display* display;
    XImage* image = NULL;
    if(mpDisplay == NULL)
    {
        mpDisplay = XOpenDisplay(NULL);
    }
    display = (Display *)mpDisplay;

    int screenWidth = 0, screenHeight = 0;
    screenWidth = XDisplayWidth(display, 0);
    screenHeight = XDisplayHeight(display, 0);

    image = XGetImage(display,
                      DefaultRootWindow( display ),
                      0, 0, screenWidth, screenHeight, AllPlanes, ZPixmap );

    if(image)
    {
        result = true;
        if(mpImageData == NULL)
        {
            mHeight = (unsigned int)screenHeight;
            mWidth = (unsigned int)screenWidth;
            mpImageData = new unsigned char[screenWidth*screenHeight*3];

        }
        // Extract RGB data and flip image.
        unsigned char pixelSize = image->bits_per_pixel/8;
        unsigned char* ptr1 = mpImageData;
        unsigned char* ptr2 = (unsigned char *)image->data;
        for(unsigned int i = 0; i < mHeight*mWidth; i++)
        {
            ptr1[2] = ptr2[2];
            ptr1[1] = ptr2[1];
            ptr1[0] = ptr2[0];
            ptr1 += 3;
            ptr2 += 4;
        }

        XDestroyImage(image);
    }

#endif

    return result;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Takes a screen grab calling CaptureScreen, then returns a copy
///   of the data.
///
///   \return Valid screen shot data on success.
///
////////////////////////////////////////////////////////////////////////////////////
ScreenCapture::Screenshot ScreenCapture::GetScreenshot()
{

    if(CaptureScreen())
    {
        return Screenshot(mpImageData, mWidth, mHeight);
    }

    return Screenshot();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Gets the current screen resolution.
///
///   \param[out] width Screen width in pixels.
///   \param[out] height Screen height in pixels.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::GetScreenResolution(unsigned int& width, unsigned int& height, int screen)
{
#ifdef WIN32
    RECT desktop;
    desktop.left = 0; 
    desktop.top = 0; 
    if(screen == -1)
    {
        //taken from http://msdn.microsoft.com/en-us/library/dd162729(v=vs.85).aspx
        desktop.right = GetSystemMetrics (SM_CXVIRTUALSCREEN); 
        desktop.bottom = GetSystemMetrics (SM_CYVIRTUALSCREEN);
    }
    else
    {
        // Get a handle to the desktop window
        //const HWND hDesktop = GetDesktopWindow();
        // Get the size of screen to the variable desktop
        //GetWindowRect(hDesktop, &desktop);
        // The top left corner will have coordinates (0,0)
        // and the bottom right corner will have coordinates
        // (horizontal, vertical)
        SystemParametersInfo(SPI_SETWORKAREA,screen,&desktop,NULL);
    }
    width = (unsigned int)desktop.right;
    height = (unsigned int)desktop.bottom;
    return true;

#else
    Display* display;
    display = XOpenDisplay(NULL);
    width = (unsigned int)XDisplayWidth(display, 0);
    height = (unsigned int)XDisplayHeight(display, 0);
    XCloseDisplay(display);
    return false;
#endif
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Saves any captured image data to disk as a .jpeg file.  If
///   continuous capture is enabled, this method will fail.
///
///   \param[in] filename The filename to save image to.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::SaveImage(const std::string& filename) const
{
    if(mpImageData && mCaptureThread.IsThreadActive() == false)
    {
        std::string saveName = filename;

        if(strstr(saveName.c_str(), ".jpg") == NULL && strstr(saveName.c_str(), ".JPG") == NULL)
        {
            saveName += ".jpg";
        }

        unsigned int compressedSize = 0;
        CxUtils::JPEG::Compressor* compressor = ((CxUtils::JPEG::Compressor*)(&mJPEGCompressor));
        compressor->CompressImage((unsigned short)mWidth,
                                 (unsigned short)mHeight,
                                 3,
                                 mpImageData,
                                 (unsigned char **)&mpCompressionBuffer,
                                 (unsigned int *)&mCompressionBufferSize,
                                 &compressedSize);

        FILE* fp = fopen(saveName.c_str(), "w+b");
        unsigned int totalWritten = 0;
        size_t written = 0;
        written = fwrite(mpCompressionBuffer, 1, compressedSize, fp);
        fclose(fp);

        if(totalWritten == compressedSize)
        {
            return true;
        }
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets a scale factor when continuous screen shot capture is running.
///          This is used when you don't need full size screen shots, but may
///          slow down writing, etc.
///
///   \param[in] scale How much to scale image.  1.0 == full size, .5 = half size,
///                    etc.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::SetScaleFactor(const double scale)
{
    if(mWriterThread.IsThreadActive() == false && scale > 0.0 && scale <= 1.0)
    {
        mScaleFactor = scale;
        return true;
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief This method creates threads which will continuously capture
///   screen shots, timestamp them, and save them to a directory with a specific
///   ID value.
///
///   \param[in] directory Folder to save images to.
///   \param[in] id ID number to tag image file names with.
///   \param[in] timeLimit How long to log screen caputres for (minutes.seconds
///                        format).  If a value greater than 0 is used, then
///                        the capture thread will exit when this time
///                        expires.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::StartCaptureThread(const std::string& directory, unsigned int id, const double timeLimit)
{
    StopCaptureThread();
    while(mWriterThread.IsThreadActive())
    {
        CxUtils::SleepMs(1);
    }
    mDirectory = directory;
    mID = id;
    double seconds1 = 60.0*((unsigned int)(timeLimit));
    double seconds2 = (timeLimit - ((unsigned int)(timeLimit)))*100.0;
    mTimeLimitSeconds =  seconds1 + seconds2;
    mCaptureQuitFlag = false;

    if(mCaptureThread.CreateThread(ScreenCapture::CaptureThread, this) &&
        mWriterThread.CreateThread(ScreenCapture::WriterThread, this))
    {
        mCaptureThread.SetThreadPriority(50);
        mWriterThread.SetThreadPriority(50);
        return true;
    }

    StopCaptureThread();
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief This method creates threads which will continuously capture
///   screen shots, timestamp them, and save them to a directory with a specific
///   ID value.
///
///   \param[in] xmlFile Filename of an XML config file which indicates
///                      frame output directory and capture time.
///   \param[in] id ID number to tag image file names with.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::StartCaptureThreadFromXML(const std::string& xmlFile, unsigned int id)
{
        TiXmlDocument file;
        if(file.LoadFile(xmlFile.c_str()))
        {
            TiXmlHandle handle(&file);
            TiXmlNode* outDir = handle.FirstChild("ScreenCapture").FirstChild("OutputDirectory").ToNode();
            TiXmlNode* timeout = handle.FirstChild("ScreenCapture").FirstChild("CaptureTime").ToNode();
            if(outDir && outDir->FirstChild() &&
               timeout->FirstChild())
            {
                return StartCaptureThread(outDir->FirstChild()->Value(), id, atof(timeout->FirstChild()->Value()));
            }
        }
        return false;
}

////////////////////////////////////////////////////////////////////////////////////
///
///   \brief This method creates threads which will continuously capture
///   screen shots, timestamp them, and save them to a directory with a specific
///   ID value.
///
///   \param[in] xmlFile Filename of an XML config file which indicates
///                      frame output directory and capture time.
///   \param[in] id ID number to tag image file names with.
///
///   \return True on success, false on failure.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::StartCaptureThreadFromXMLElement(TiXmlNode* element, unsigned int id)
{
    if (element)
    {
        TiXmlElement* outDir = element->FirstChildElement("OutputDirectory");
        TiXmlElement* timeout = element->FirstChildElement("CaptureTime");
        TiXmlElement* scale = element->FirstChildElement("Scale");
        if(outDir && outDir->FirstChild() &&
            timeout->FirstChild())
        {
            if(scale && scale->FirstChild())
            {
                SetScaleFactor(atof(scale->FirstChild()->Value()));
            }
            return StartCaptureThread(outDir->FirstChild()->Value(), id, atof(timeout->FirstChild()->Value()));
        }
    }
    return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Stops continuous capture of screen shots.
///
////////////////////////////////////////////////////////////////////////////////////
void ScreenCapture::StopCaptureThread()
{
    mCaptureQuitFlag = true;
    mCaptureThread.StopThread();
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \return True if continuous capture is active, otherwise false.
///
////////////////////////////////////////////////////////////////////////////////////
bool ScreenCapture::IsActive() const
{
     if(mCaptureThread.IsThreadActive() == true ||
        mWriterThread.IsThreadActive() == true)
        {
            return true;
        }
        return false;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Function run within a thread whic continuously captures screen
///   shots and passes them to a queue for writing by another thread.
///
////////////////////////////////////////////////////////////////////////////////////
void ScreenCapture::CaptureThread(void* arg)
{
    ScreenCapture* capture = (ScreenCapture*)arg;
    capture->mCaptureCount = 0;
    unsigned int toWrite = 0;

    bool timeOutFlag = false;
    double startTimeSeconds = CxUtils::Timer::GetTimeSeconds();
    if(capture->mTimeLimitSeconds > .001)
    {
        timeOutFlag = true;
    }
    while(!capture->mCaptureThread.QuitThreadFlag())
    {
        capture->mScreenshotsMutex.Lock();
        toWrite = (unsigned int)capture->mScreenshots.size();
        capture->mScreenshotsMutex.Unlock();
        if(toWrite < 10 && capture->CaptureScreen(capture->mScreenNumber))
        {
            capture->mScreenshotsMutex.Lock();
            capture->mScreenshots.push_back(new Screenshot(capture->mpImageData, capture->mWidth, capture->mHeight));
            capture->mScreenshotsMutex.Unlock();
            capture->mCaptureCount++;
        }
        // Check to see if we need to stop capture.
        if(timeOutFlag == true &&
           CxUtils::Timer::GetTimeSeconds() - startTimeSeconds >= capture->mTimeLimitSeconds + .5)
        {
            break;
        }
        CxUtils::SleepMs(capture->mCaptureDelayMs);
    }

    capture->mCaptureQuitFlag = true;
}


////////////////////////////////////////////////////////////////////////////////////
///
///   \brief Function run within a thread which continuously pulls captured
///   images from a queue and writes them to disk.
///
////////////////////////////////////////////////////////////////////////////////////
void ScreenCapture::WriterThread(void* arg)
{
    ScreenCapture* capture = (ScreenCapture*)arg;
    Screenshot* image = NULL;
    Screenshot scaledImage;
    Screenshot::List::iterator itr;

    char filename[512];
    unsigned int count = 0;
    // Make sure the directory exists.
    if(capture->mDirectory.empty() == false)
    {
        CxUtils::FileIO::CreateDir(capture->mDirectory);
    }

    unsigned int toWrite = 0;
    while(!capture->mCaptureQuitFlag || toWrite > 0)
    {
        if(image)
        {
            delete image;
            image = NULL;
        }

        // Try pop an image from screenshot queue
        capture->mScreenshotsMutex.Lock();
        itr = capture->mScreenshots.begin();
        if(itr != capture->mScreenshots.end())
        {
            image = *itr;
            *itr = NULL;
            capture->mScreenshots.erase(itr);
        }
        toWrite = (unsigned int)capture->mScreenshots.size();
        capture->mScreenshotsMutex.Unlock();

        // Save image to disk.

        if(image)
        {
            if(capture->mDirectory.empty())
            {
                sprintf(filename,
                        "%d - %02d.%02d.%02d.%03d - %.03lf.jpg",
                        capture->mID,
                        image->mTimeStamp.mHour,
                        image->mTimeStamp.mMinute,
                        image->mTimeStamp.mSecond,
                        image->mTimeStamp.mMilliseconds,
                        image->mTimeStamp.ToSeconds());
            }
            else
            {
                sprintf(filename,
                        "%s/%d - %02d.%02d.%02d.%03d - %.03lf.jpg",
                        capture->mDirectory.c_str(),
                        capture->mID,
                        image->mTimeStamp.mHour,
                        image->mTimeStamp.mMinute,
                        image->mTimeStamp.mSecond,
                        image->mTimeStamp.mMilliseconds,
                        image->mTimeStamp.ToSeconds());
            }
            unsigned int jpegSize = 0;
            if(capture->mScaleFactor >= 1.0)
            {
                capture->mJPEGCompressor.CompressImage(image->mWidth, image->mHeight, 3, image->mpImage,
                                                       &capture->mpCompressionBuffer, &capture->mCompressionBufferSize, &jpegSize);
                FILE* fp = fopen(filename, "wb");
                fwrite(capture->mpCompressionBuffer, 1, jpegSize, fp);
                fclose(fp);
            }
            else
            {
                scaledImage.Create(image->mWidth,
                                   image->mHeight,
                                   image->mChannels,
                                   image->mpImage,
                                   capture->mScaleFactor,
                                   false);
                if(scaledImage.mpImage)
                {
                    capture->mJPEGCompressor.CompressImage(scaledImage.mWidth, 
                                                           scaledImage.mHeight, 3, 
                                                           scaledImage.mpImage,
                                                           &capture->mpCompressionBuffer, 
                                                           &capture->mCompressionBufferSize, 
                                                           &jpegSize);
                    FILE* fp = fopen(filename, "wb");
                    fwrite(capture->mpCompressionBuffer, 1, jpegSize, fp);
                    fclose(fp);
                }
            }

            delete image;
            image = NULL;
        }
        CxUtils::SleepMs(1);
    }
    // Clean up memory.
    if(image)
    {
        delete image;
        image = NULL;
    }
}

/*  End of File */
