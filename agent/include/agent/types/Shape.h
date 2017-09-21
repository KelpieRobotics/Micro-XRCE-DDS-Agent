// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file Shape.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _Shape_H_
#define _Shape_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(Shape_SOURCE)
#define Shape_DllAPI __declspec( dllexport )
#else
#define Shape_DllAPI __declspec( dllimport )
#endif // Shape_SOURCE
#else
#define Shape_DllAPI
#endif
#else
#define Shape_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure ShapeType defined by the user in the IDL file.
 * @ingroup SHAPE
 */

class ShapeType
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport ShapeType();
    
    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~ShapeType();
    
    /*!
     * @brief Copy constructor.
     * @param x Reference to the object ShapeType that will be copied.
     */
    eProsima_user_DllExport ShapeType(const ShapeType &x);
    
    /*!
     * @brief Move constructor.
     * @param x Reference to the object ShapeType that will be copied.
     */
    eProsima_user_DllExport ShapeType(ShapeType &&x);
    
    /*!
     * @brief Copy assignment.
     * @param x Reference to the object ShapeType that will be copied.
     */
    eProsima_user_DllExport ShapeType& operator=(const ShapeType &x);
    
    /*!
     * @brief Move assignment.
     * @param x Reference to the object ShapeType that will be copied.
     */
    eProsima_user_DllExport ShapeType& operator=(ShapeType &&x);
    
    /*!
     * @brief This function copies the value in member color
     * @param _color New value to be copied in member color
     */
    inline eProsima_user_DllExport void color(const std::string &_color)
    {
        m_color = _color;
    }

    /*!
     * @brief This function moves the value in member color
     * @param _color New value to be moved in member color
     */
    inline eProsima_user_DllExport void color(std::string &&_color)
    {
        m_color = std::move(_color);
    }

    /*!
     * @brief This function returns a constant reference to member color
     * @return Constant reference to member color
     */
    inline eProsima_user_DllExport const std::string& color() const
    {
        return m_color;
    }

    /*!
     * @brief This function returns a reference to member color
     * @return Reference to member color
     */
    inline eProsima_user_DllExport std::string& color()
    {
        return m_color;
    }
    /*!
     * @brief This function sets a value in member x
     * @param _x New value for member x
     */
    inline eProsima_user_DllExport void x(int32_t _x)
    {
        m_x = _x;
    }

    /*!
     * @brief This function returns the value of member x
     * @return Value of member x
     */
    inline eProsima_user_DllExport int32_t x() const
    {
        return m_x;
    }

    /*!
     * @brief This function returns a reference to member x
     * @return Reference to member x
     */
    inline eProsima_user_DllExport int32_t& x()
    {
        return m_x;
    }
    /*!
     * @brief This function sets a value in member y
     * @param _y New value for member y
     */
    inline eProsima_user_DllExport void y(int32_t _y)
    {
        m_y = _y;
    }

    /*!
     * @brief This function returns the value of member y
     * @return Value of member y
     */
    inline eProsima_user_DllExport int32_t y() const
    {
        return m_y;
    }

    /*!
     * @brief This function returns a reference to member y
     * @return Reference to member y
     */
    inline eProsima_user_DllExport int32_t& y()
    {
        return m_y;
    }
    /*!
     * @brief This function sets a value in member shapesize
     * @param _shapesize New value for member shapesize
     */
    inline eProsima_user_DllExport void shapesize(int32_t _shapesize)
    {
        m_shapesize = _shapesize;
    }

    /*!
     * @brief This function returns the value of member shapesize
     * @return Value of member shapesize
     */
    inline eProsima_user_DllExport int32_t shapesize() const
    {
        return m_shapesize;
    }

    /*!
     * @brief This function returns a reference to member shapesize
     * @return Reference to member shapesize
     */
    inline eProsima_user_DllExport int32_t& shapesize()
    {
        return m_shapesize;
    }
    
    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const ShapeType& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;
    
private:
    std::string m_color;
    int32_t m_x;
    int32_t m_y;
    int32_t m_shapesize;
};

#endif // _Shape_H_
