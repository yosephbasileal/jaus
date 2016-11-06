/*==========================================================================

    Filename: imagepanel.h

    Copyright (C) 2010  Daniel Barber
                        Brian C. Becker                        
                        Robotics Lab at UCF
                        http://robotics.ucf.edu

    Description:
    ------------------------------------------------------------------------
    This is a control designed to display an image or flickerless realtime
    video. Once you give it an image to display, it continues
    to display that image until you set another one. The transition 
    between images is realtime when optimized and almost flickerless.

    License:
    ------------------------------------------------------------------------
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, 
    Boston, MA  02110-1301  USA

===========================================================================*/
#include "ImagePanel.h"
#include <wx/event.h>

BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)

// some useful events
/*
 EVT_MOTION(wxImagePanel::mouseMoved)
 EVT_LEFT_DOWN(wxImagePanel::mouseDown)
 EVT_LEFT_UP(wxImagePanel::mouseReleased)
 EVT_RIGHT_DOWN(wxImagePanel::rightClick)
 EVT_LEAVE_WINDOW(wxImagePanel::mouseLeftWindow)
 EVT_KEY_DOWN(wxImagePanel::keyPressed)
 EVT_KEY_UP(wxImagePanel::keyReleased)
 EVT_MOUSEWHEEL(wxImagePanel::mouseWheelMoved)
 */

// catch paint events
EVT_PAINT(wxImagePanel::PaintEvent)
EVT_ERASE_BACKGROUND(wxImagePanel::OnBackgroundErase)
EVT_SIZE(wxImagePanel::OnSize)

END_EVENT_TABLE()


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Create a wxImagePanel with a border
///
///   Constructor that takes the standard wxWidget's parameters and a border
///   and creates the wxImagePanel. See Create for parameter details.
///
///   \param[in] parent Parent control.
///
//////////////////////////////////////////////////////////////////////////////
wxImagePanel::wxImagePanel(wxWindow *parent,
                 wxWindowID winid,
                 const wxPoint& pos,
                 const wxSize& size ,
                 long style,
                 const wxString& name)
                 : wxPanel(parent, winid, pos, size, style, name)
{
    wxInitAllImageHandlers();
    if(size.GetWidth() <= 0 || size.GetHeight() <= 0)
    {
        mImage.Create(1, 1, true);
    }
    else
    {
        mImage.Create(size.GetWidth(), size.GetHeight(), true);
    }
    mResizeFlag = true;
    SetBackgroundColour( wxColour( 0, 0, 0 ) );
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the image to display in panel.
///
///   \param[in] file Filename of image to load.
///
///   \return True on success, false on failure.
///
//////////////////////////////////////////////////////////////////////////////
bool wxImagePanel::SetImage(const wxString& file)
{
    bool result = false;
    if(mImage.LoadFile(file))
    {
        result = SetImage(mImage);
    }

    return result;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the image to display in panel. Image must have 3
///          channels.
///
///   \param[in] data Pointer to RGB image data.
///   \param[in] width Width of the image.
///   \param[in] height Height of the image.
///
///   \return True on success, false on failure.
///
//////////////////////////////////////////////////////////////////////////////
bool wxImagePanel::SetImage(const unsigned char *data, 
                            const int width,
                            const int height)
{
    if(data && width > 0 && height > 0)
    {
        mThreadProtection.Lock();
        if(mImage.GetWidth() != width || mImage.GetHeight() != height)
        {
            mImage.Create(width, height);
        }
        if(mImage.Ok())
        {
            // Copy the data to the buffer
            memcpy(mImage.GetData(), data, width*height*3);
        }
        mThreadProtection.Unlock();

        Refresh(true);
        return true;
    }
    return false;
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Sets the image to display in panel.
///
///   \param[in] image Image data to display.
///
///   \return True on success, false on failure.
///
//////////////////////////////////////////////////////////////////////////////
bool wxImagePanel::SetImage(const wxImage& image)
{
    return SetImage((const unsigned char *)image.GetData(),
                    image.GetWidth(),
                    image.GetHeight());
}


// Clears the data.
void wxImagePanel::ClearImage()
{
    mThreadProtection.Lock();
    mResizeFlag = true;
    if(GetParent())
    {
        mImage.Create(GetParent()->GetSize().GetWidth(),
                      GetParent()->GetSize().GetHeight(),
                      true);
    }
    else
    {
        mImage.Create(10, 10, true);
    }
    mThreadProtection.Unlock();

    Refresh(true);
}


/** Checks for size changes. */
void wxImagePanel::OnSize(wxSizeEvent& event)
{
    mResizeFlag = true;
    Refresh(true);
    event.Skip();
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called on a paint event to re-draw the screen.
///
//////////////////////////////////////////////////////////////////////////////
void wxImagePanel::PaintEvent(wxPaintEvent & evt)
{
    wxPaintDC dc(this);
    Render(dc);
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called on a paint event to re-draw the screen.
///
//////////////////////////////////////////////////////////////////////////////
void wxImagePanel::PaintNow()
{
    // depending on your system you may need to look at double-buffered dcs
    wxBufferedPaintDC dc(this);
    PrepareDC(dc);
    Render(dc);
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called on a paint event to re-draw the screen.
///
//////////////////////////////////////////////////////////////////////////////
void wxImagePanel::Render(wxDC&  dc)
{
    int neww, newh;
    this->GetClientSize(&neww, &newh);
    mThreadProtection.Lock();
    if(mImage.GetWidth() > 0 && mImage.GetHeight() > 0)
    {
        double x, y;
        x = (double)neww/mImage.GetWidth();
        y = (double)newh/mImage.GetHeight();
        
        //  Find the smallest scale value
        //  to scale evenly in both x and y directions
        if (x < y)
            y = x;
        else
            x = y;

        if(mImage.GetWidth() == neww && mImage.GetHeight() == newh)
        {
            mResizedImage = wxBitmap(mImage);
        }
        else
        {
            mResizedImage = wxBitmap( mImage.Scale( (int)(mImage.GetWidth()*x),
                                                    (int)(mImage.GetHeight()*y)) );
        }
        mThreadProtection.Unlock();
        dc.SetBackground(wxBrush(GetBackgroundColour(), wxSOLID));
        if(mResizeFlag)
        {
            dc.Clear();
            mResizeFlag = false;
        }
        dc.DrawBitmap( mResizedImage, 
                       (int)((neww - mResizedImage.GetWidth())/2.0), 
                       (int)((newh - mResizedImage.GetHeight())/2.0), false );
    }
    else
    {
        mThreadProtection.Unlock();
    }
}


//////////////////////////////////////////////////////////////////////////////
///
///   \brief Method called on background erase event.
///
//////////////////////////////////////////////////////////////////////////////
void wxImagePanel::OnBackgroundErase(wxEraseEvent& event)
{

}

/*  End of File */
