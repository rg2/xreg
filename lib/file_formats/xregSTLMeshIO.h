/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 * \brief I/O routines for meshes in STL format.
 *
 * Routines for reading and writing meshes in STL format. Currently, ASCII
 * STL files are NOT supported.
 **/

#ifndef XREGSTLMESHIO_H_
#define XREGSTLMESHIO_H_

#include <string>
#include <iosfwd>

#include "xregMesh.h"

namespace xreg
{

// START FORWARD DECLARATIONS

class InputStream;
class OutputStream;

// STOP FORWARD DECLARATIONS

/**
 * @brief Reads an STL file.
 *
 * Reads an STL file from disk and populates a triangular mesh structure. Since
 * STL files duplicate vertices, this routine identifies those duplicates and
 * stores a unique vertex list by default.
 * @param path Path on disk to a STL file to read
 * @param mesh Triangular mesh structure to populate
 * @param remove_dups (Optional) Boolean flag indicating that duplicate vertices
 *        should be identified before populating the triangular mesh structure;
 *        defaults to true
 **/
TriMesh ReadSTLMesh(const std::string& path, const bool remove_dups = true);

/**
 * @brief Reads an STL file.
 *
 * Reads an STL file from an input stream and populates a triangular mesh
 * structure. Since STL files duplicate vertices, this routine identifies those
 * duplicates and stores a unique vertex list by default.
 * @param in Input stream to read
 * @param mesh Triangular mesh structure to populate
 * @param remove_dups (Optional) Boolean flag indicating that duplicate vertices
 *        should be identified before populating the triangular mesh structure;
 *        defaults to true
 **/
TriMesh ReadSTLMesh(std::istream& in, const bool remove_dups = true);

/**
 * @brief Reads an STL file.
 *
 * Reads an STL file from an input stream and populates a triangular mesh
 * structure. Since STL files duplicate vertices, this routine identifies those
 * duplicates and stores a unique vertex list by default.
 * @param in Input stream to read
 * @param mesh Triangular mesh structure to populate
 * @param remove_dups (Optional) Boolean flag indicating that duplicate vertices
 *        should be identified before populating the triangular mesh structure;
 *        defaults to true
 **/
TriMesh ReadSTLMesh(InputStream& in, const bool remove_dups = true);

/**
 * @brief Writes a mesh to a file in STL format.
 *
 * Writes a triangular mesh structure to disk in STL format.
 * @param path Path on disk to a STL file to write to
 * @param mesh Triangular mesh structure to write to disk
 * @param ascii (optional) Boolean flag indicating that the STL file should be
 *        written in ASCII format; defaults to false
 * @param ascii_name (optional) String that serves as an identifier for this mesh
 *        when writing STL in ASCII format; defaults to the empty string
 **/
void WriteSTLMesh(const std::string& path, const TriMesh& mesh,
                  const bool ascii = false, const std::string& ascii_name = "");

/**
 * @brief Writes a mesh to a xreg::OutputStream in STL format.
 *
 * Writes a triangular mesh structure to disk in STL format.
 * @param out Output stream to write to
 * @param mesh Triangular mesh structure to write to disk
 * @param ascii (optional) Boolean flag indicating that the STL file should be
 *        written in ASCII format; defaults to false
 * @param ascii_name (optional) String that serves as an identifier for this mesh
 *        when writing STL in ASCII format; defaults to the empty string
 **/
void WriteSTLMesh(OutputStream& out, const TriMesh& mesh,
                  const bool ascii = false, const std::string& ascii_name = "");

/**
 * @brief Writes a mesh to a std::ostream in STL format.
 *
 * Writes a triangular mesh structure to disk in STL format.
 * @param out Output stream to write to
 * @param mesh Triangular mesh structure to write to disk
 * @param ascii (optional) Boolean flag indicating that the STL file should be
 *        written in ASCII format; defaults to false
 * @param ascii_name (optional) String that serves as an identifier for this mesh
 *        when writing STL in ASCII format; defaults to the empty string
 **/
void WriteSTLMesh(std::ostream& out, const TriMesh& mesh,
                  const bool ascii = false, const std::string& ascii_name = "");

}  // xreg

#endif
