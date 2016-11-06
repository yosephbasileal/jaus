////////////////////////////////////////////////////////////////////////////////////
///
///  \file XmlConfig.cpp
///  \brief Helper class to simplify getting some variables and data in
///         XML files dynamically.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 3/6/2013
///  <br>Copyright (c) 2013
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
#include "cxutils/xml/XmlConfig.h"
#include "cxutils/FileIO.h"
#include <tinyxml.h>
#include <exception>
#include <stdexcept>
#include <sstream>

using namespace CxUtils;

XmlConfig::XmlConfig()
{
    mpDocument = new TiXmlDocument();
    mpHandle = NULL;
    mOpenFlag = false;
    mFileTimeSeconds = 0;
}


XmlConfig::XmlConfig(const std::string& inFile)
{
    mpDocument = new TiXmlDocument();
    mpHandle = NULL;
    mOpenFlag = false;
    mFileTimeSeconds = 0;
    LoadFile(inFile);
}

XmlConfig::~XmlConfig()
{
    delete mpDocument;
    if(mpHandle)
    {
        delete mpHandle;
    }
    mpHandle = NULL;
}

bool XmlConfig::IsOpen() const
{
    return mOpenFlag;
}

bool XmlConfig::LoadFile(const std::string& filename)
{
    mOpenFlag = false;
    if(mpHandle)
    {
        delete mpHandle;
    }
    mpHandle = NULL;

    if(mpDocument->LoadFile(filename))
    {
        mFilename = filename;
        mpHandle = new TiXmlHandle(mpDocument);
        mOpenFlag = true;
        mFileTimeSeconds = CxUtils::FileIO::GetFileTime(filename);
        return true;
    }
    return false;
}


class XmlConfigVisitor : public TiXmlVisitor
{
public:
    XmlConfigVisitor(const std::string& desiredName)
        : mDesiredElement(desiredName)
    {
        mpElement = NULL;
    }
    virtual bool VisitExit(const TiXmlElement& e)
    {
        if(e.Value() == mDesiredElement)
        {
            mpElement = (TiXmlElement* )&e;
            return false;
        }
        return true;
    }
    std::string mDesiredElement;
    TiXmlElement* mpElement;
};


/** 
    \brief Gets the string of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         std::string& value,
                         const bool required) const
{
    if(mOpenFlag == false)
    {
        if(required)
        {
            std::stringstream msg;
            msg << "XmlConfig:ERROR - No Document Loaded.";
            throw std::runtime_error(msg.str().c_str());
        }
        return false;
    }

    unsigned int ftime = 
        CxUtils::FileIO::GetFileTime(mFilename);
    if(ftime != mFileTimeSeconds)
    {
        XmlConfig* ptr = (XmlConfig*)this;
        ptr->LoadFile(mFilename);
    }

    // Tokenize the string.
    std::vector<std::string> nodeTokens;
    std::vector<std::string> attrTokens;
    std::string attrName;
    nodeTokens = FileIO::Tokenize(formatString, ".");

    attrTokens = FileIO::Tokenize(nodeTokens.back(), "@");

    if(attrTokens.size() > 1)
    {
        // No attribute tokens.
        attrName = attrTokens.back();
        nodeTokens.back() = attrTokens.front();
    }

    TiXmlElement* currentNode = mpDocument->FirstChildElement();
    
    bool found = false;
    // Node find the first node.
    std::vector<std::string>::iterator nodeName;
    for(nodeName = nodeTokens.begin();
        nodeName != nodeTokens.end() && currentNode != NULL;
        nodeName++)
    {
        XmlConfigVisitor visit(*nodeName);
        currentNode->Accept(&visit);
        if(visit.mpElement == NULL)
        {
            // Failed to find node
            break;
        }
        currentNode = visit.mpElement;
        // Is this the last node to search?
        if(nodeName + 1 == nodeTokens.end())
        {
            if(attrName.empty())
            {
                value = currentNode->GetText();
                if(value.empty() == false)
                {
                    found = true;
                }
            }
            else
            {
                const std::string* s = currentNode->Attribute(attrName);
                if(s != NULL)
                {
                    value = *s;
                    found = true;
                }
            }
            break;
        }
    }

    if(required && found == false)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return found;
}


/** 
    \brief Gets the element pointer to the node described by
    the format string.

    \param formatString Formatted string to find value from.
                        "Node1.Node3" or
                        "Node1.Node4"."

    \return Pointer to element, NULL if not found.
*/
TiXmlElement* XmlConfig::GetElement(const std::string& formatString) const
{
    if(mOpenFlag == false)
    {
        return false;
    }

    unsigned int ftime = 
        CxUtils::FileIO::GetFileTime(mFilename);
    if(ftime != mFileTimeSeconds)
    {
        XmlConfig* ptr = (XmlConfig*)this;
        ptr->LoadFile(mFilename);
    }

    // Tokenize the string.
    std::vector<std::string> nodeTokens;
    std::vector<std::string> attrTokens;
    std::string attrName;
    nodeTokens = FileIO::Tokenize(formatString, ".");

    attrTokens = FileIO::Tokenize(nodeTokens.back(), "@");

    if(attrTokens.size() > 1)
    {
        // No attribute tokens.
        attrName = attrTokens.back();
        nodeTokens.back() = attrTokens.front();
    }

    TiXmlElement* currentNode = mpDocument->FirstChildElement();
    
    bool found = false;
    // Node find the first node.
    std::vector<std::string>::iterator nodeName;
    for(nodeName = nodeTokens.begin();
        nodeName != nodeTokens.end() && currentNode != NULL;
        nodeName++)
    {
        XmlConfigVisitor visit(*nodeName);
        currentNode->Accept(&visit);
        if(visit.mpElement == NULL)
        {
            // Failed to find node
            break;
        }
        currentNode = visit.mpElement;
        // Is this the last node to search?
        if(nodeName + 1 == nodeTokens.end())
        {
            return currentNode->ToElement();
        }
    }

    return NULL;
}


/** 
    \brief Gets the value of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         unsigned char& value,
                         const bool required) const
{
    std::string s;
    if(GetValue(formatString, s, required))
    {
        value = (unsigned char)atoi(s.c_str());
        return true;
    }

    if(required)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return false;
}


/** 
    \brief Gets the value of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         int& value,
                         const bool required) const
{
    std::string s;
    if(GetValue(formatString, s, required))
    {
        value = atoi(s.c_str());
        return true;
    }

    if(required)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return false;
}


/** 
    \brief Gets the value of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         unsigned int& value,
                         const bool required) const
{
    std::string s;
    if(GetValue(formatString, s, required))
    {
        value = (unsigned int)atoi(s.c_str());
        return true;
    }

    if(required)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return false;
}


/** 
    \brief Gets the value of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         float& value,
                         const bool required) const
{
    std::string s;
    if(GetValue(formatString, s, required))
    {
        value = (float)atof(s.c_str());
        return true;
    }

    if(required)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return false;
}


/** 
    \brief Gets the value of an attribute or node text given
    a formatted string.

    \param formatString Formatted string to find value from.
                        "Node1.Node2@attribute_name" or
                        "Node1.Node2"."
    \param value Value retrieved. Only modified 
                 if new value found.
    \param required If true, an exception is thrown if
                    unable to find value.

    \return True on success, false on failure. 
*/
bool XmlConfig::GetValue(const std::string& formatString,
                         double& value,
                         const bool required) const
{
    std::string s;
    if(GetValue(formatString, s, required))
    {
        value = (double)atof(s.c_str());
        return true;
    }

    if(required)
    {
        std::stringstream msg;
        msg << "XmlConfig:ERROR - " << formatString;
        throw std::runtime_error(msg.str().c_str());
    }

    return false;
}



/*  End of File */
