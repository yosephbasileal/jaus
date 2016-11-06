////////////////////////////////////////////////////////////////////////////////////
///
///  \file XmlConfig.h
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
#ifndef __CXUTILS_XML_XML_CONFIG_H
#define __CXUTILS_XML_XML_CONFIG_H

#include <vector>
#include <string>
#include "cxutils/CxBase.h"

class TiXmlDocument;    // Forward declarations.
class TiXmlHandle;      // Forward declarations.
class TiXmlElement;     // Forward declarations.

namespace CxUtils
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class XmlConfig
    ///   \brief Class used for simplified getting of data from XML files.
    ///
    ///   A format string uses the following format.
    ///   "ParentNode.ChildNode.Child.node_name" - Get value at lowest node.
    ///   "ParentNode.ChildNode@attribute_name" - Gets attribute value at Child Node.
    ///
    ///   For example:
    ///   <Parent name="test">Value<\Parent>
    ///   Format string of "Parent@name" would get the attribute "test," and 
    ///   a string of "Parent" would get the value "Value."
    ///
    ///   This class will also search for find the first node matching the name of 
    ///   the first node in your format string. For example, if the root node is
    ///   Root, and you pass a string of "Parent.Child," the functions will search
    ///   past the root node to find the "Parent" node for you, so you do not need
    ///   to put "Root.Parent.Child" in your search string.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class CX_UTILS_DLL XmlConfig
    {
    public:
        XmlConfig();
        XmlConfig(const std::string& inFile);
        ~XmlConfig();
        bool IsOpen() const;
        bool LoadFile(const std::string& filename);
        std::string GetFilename() const { return mFilename; }
        bool GetValue(const std::string& formatString,
                      std::string& value,
                      const bool required = false) const;
        bool GetValue(const std::string& formatString,
                      unsigned char& value,
                      const bool required = false) const;
        bool GetValue(const std::string& formatString,
                      int& value,
                      const bool required = false) const;
        bool GetValue(const std::string& formatString,
                      unsigned int& value,
                      const bool required = false) const;
        bool GetValue(const std::string& formatString,
                      float& value,
                      const bool required = false) const;
        bool GetValue(const std::string& formatString,
                      double& value,
                      const bool required = false) const;
        TiXmlElement* GetElement(const std::string& formatString) const;
        TiXmlHandle* GetHandle() { return mpHandle; }
    protected:
        std::string mFilename;      ///< Name of XML file.
        TiXmlDocument* mpDocument;  ///< TinyXML structures.
        TiXmlHandle* mpHandle;      ///< TinyXML structures.
        bool mOpenFlag;             ///< Is coudment open?
        unsigned int mFileTimeSeconds;     ///< Time of last file modificaiton.
    };
}
#endif

/* End of File*/
