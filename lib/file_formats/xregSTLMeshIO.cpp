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

#include "xregSTLMeshIO.h"

#include <fstream>

#include "xregExceptionUtils.h"
#include "xregStdStreamUtils.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregAssert.h"

namespace
{

using namespace xreg;

TriMesh ReadSTLMeshASCII(InputStream& in)
{
  // I'm not very proud of this code, but I needed something quick.

  using Mesh     = TriMesh;
  using Vertex   = Mesh::Vertex;
  using Triangle = Mesh::Triangle;

  Mesh mesh;

  std::string tmp_str;
  Vertex tmp_data;
  Triangle tmp_face;

  size_type num_verts = 0;

  bool keep_parsing = true;

  while (keep_parsing)
  {
    tmp_str = in.read_ascii_token();
    if (tmp_str == "facet")
    {
      tmp_str = in.read_ascii_token();
      if (tmp_str == "normal")
      {
        tmp_data[0] = StringCast<CoordScalar>(in.read_ascii_token());
        tmp_data[1] = StringCast<CoordScalar>(in.read_ascii_token());
        tmp_data[2] = StringCast<CoordScalar>(in.read_ascii_token());

        // flip the direction of the normal
        tmp_data *= -1;

        mesh.normals.push_back(tmp_data);

        tmp_str = in.read_ascii_token();
        if (tmp_str == "outer")
        {
          tmp_str = in.read_ascii_token();
          if (tmp_str == "loop")
          {
            tmp_str = in.read_ascii_token();
            if (tmp_str == "vertex")
            {
              tmp_data[0] = StringCast<CoordScalar>(in.read_ascii_token());
              tmp_data[1] = StringCast<CoordScalar>(in.read_ascii_token());
              tmp_data[2] = StringCast<CoordScalar>(in.read_ascii_token());
              mesh.vertices.push_back(tmp_data);

              tmp_str = in.read_ascii_token();
              if (tmp_str == "vertex")
              {
                tmp_data[0] = StringCast<CoordScalar>(in.read_ascii_token());
                tmp_data[1] = StringCast<CoordScalar>(in.read_ascii_token());
                tmp_data[2] = StringCast<CoordScalar>(in.read_ascii_token());
                mesh.vertices.push_back(tmp_data);

                tmp_str = in.read_ascii_token();
                if (tmp_str == "vertex")
                {
                  tmp_data[0] = StringCast<CoordScalar>(in.read_ascii_token());
                  tmp_data[1] = StringCast<CoordScalar>(in.read_ascii_token());
                  tmp_data[2] = StringCast<CoordScalar>(in.read_ascii_token());
                  mesh.vertices.push_back(tmp_data);

                  tmp_str = in.read_ascii_token();
                  if (tmp_str == "endloop")
                  {
                    tmp_str = in.read_ascii_token();
                    if (tmp_str == "endfacet")
                    {
                      tmp_face[0] = num_verts;
                      tmp_face[1] = num_verts + 1;
                      tmp_face[2] = num_verts + 2;

                      mesh.faces.push_back(tmp_face);

                      num_verts += 3;
                    }
                    else
                    {
                      keep_parsing = false;
                    }
                  }
                  else
                  {
                    keep_parsing = false;
                  }
                }
                else
                {
                  keep_parsing = false;
                }
              }
              else
              {
                keep_parsing = false;
              }
            }
            else
            {
              keep_parsing = false;
            }
          }
          else
          {
            keep_parsing = false;
          }
        }
        else
        {
          keep_parsing = false;
        }
      }
      else
      {
        keep_parsing = false;
      }
    }
    else if (tmp_str == "endsolid")
    {
      keep_parsing = false;
    }
    else
    {
      keep_parsing = false;
    }
  }

  xregASSERT(num_verts == mesh.vertices.size());
  xregASSERT(mesh.vertices.size() == (mesh.faces.size() * 3));
  xregASSERT(mesh.faces.size() == mesh.normals.size());

  mesh.normals_valid = true;

  return mesh;
}

void WriteSTLMeshASCII(OutputStream& /*out*/, const TriMesh& /*mesh*/, const std::string& /*name*/)
{
  xregThrow("ASCII STL not currently supported!");
}

TriMesh ReadSTLMeshBinary(InputStream& in)
{
  in.set_byte_order(kLITTLE_ENDIAN);

  xreg::Stream::int32  tmp_int32 = 0;
  xreg::Stream::uint16 tmp_int16 = 0;
  float tmp_floats[12];

  in >> tmp_int32;

  const size_type num_tris = tmp_int32;
  
  TriMesh mesh;

  mesh.vertices.resize(num_tris * 3);
  mesh.faces.resize(num_tris);
  mesh.normals.resize(num_tris);

  mesh.normals_valid = true;

  for (size_type i = 0; i < num_tris; ++i)
  {
    in.read(tmp_floats, 12);
    in >> tmp_int16;

    const size_type vert_off = 3 * i;

    // negate to switch from outward normals to inward normals
    mesh.normals[i][0] = -tmp_floats[0];
    mesh.normals[i][1] = -tmp_floats[1];
    mesh.normals[i][2] = -tmp_floats[2];
    if (std::abs(mesh.normals[i].norm() - 1.0) > 1.0e-4)
    {
      if (mesh.normals_valid)
      {
        mesh.normals_valid = false;

        std::cerr << "Triangle #" << i << " has an invalid normal "
          "vector (suppressing this warning for future bad normals in this file)"
                            << std::endl;
      }
    }

    mesh.vertices[vert_off + 2][0] = tmp_floats[3];
    mesh.vertices[vert_off + 2][1] = tmp_floats[4];
    mesh.vertices[vert_off + 2][2] = tmp_floats[5];

    mesh.vertices[vert_off + 1][0] = tmp_floats[6];
    mesh.vertices[vert_off + 1][1] = tmp_floats[7];
    mesh.vertices[vert_off + 1][2] = tmp_floats[8];

    mesh.vertices[vert_off][0] = tmp_floats[9];
    mesh.vertices[vert_off][1] = tmp_floats[10];
    mesh.vertices[vert_off][2] = tmp_floats[11];

    mesh.faces[i][0] = vert_off;
    mesh.faces[i][1] = vert_off + 1;
    mesh.faces[i][2] = vert_off + 2;
  }

  return mesh;
}

void WriteSTLMeshBinary(OutputStream& out, const TriMesh& mesh)
{
  out.set_byte_order(kLITTLE_ENDIAN);

  Stream::int8 tmp_buf[80];
  memset(tmp_buf, 0, 80);

  out.write(tmp_buf, 80);

  xreg::Stream::int32 tmp_int = static_cast<xreg::Stream::int32>(mesh.faces.size());
  out << tmp_int;

  const xreg::Stream::uint16 att = 0;

  float tmp_floats[12];

  using Vertex = TriMesh::Vertex;

  Vertex normal;

  const size_type num_tris = mesh.faces.size();

  for (size_type tri_index = 0; tri_index < num_tris; ++tri_index)
  {
    const auto& tri = mesh.faces[tri_index];

    const Vertex& v1 = mesh.vertices[tri[0]];
    const Vertex& v2 = mesh.vertices[tri[1]];
    const Vertex& v3 = mesh.vertices[tri[2]];

    if (mesh.normals_valid)
    {
      // invert to switch from inward facing normals to outward facing
      tmp_floats[0] = -static_cast<float>(mesh.normals[tri_index][0]);
      tmp_floats[1] = -static_cast<float>(mesh.normals[tri_index][1]);
      tmp_floats[2] = -static_cast<float>(mesh.normals[tri_index][2]);
    }
    else
    {
      // computing outward facing normal, relying on the vertices being ordered
      // clockwise as seen from the "outside"
      try
      {
        normal = (v3 - v1).cross(v2 - v1);
        normal.normalize();  // This has thrown an exception before when using CISST, I am not certain why a divide by zero was encountered.

        tmp_floats[0] = static_cast<float>(normal[0]);
        tmp_floats[1] = static_cast<float>(normal[1]);
        tmp_floats[2] = static_cast<float>(normal[2]);
      }
      catch (...)
      {
        std::cerr << "WARNING, when writing STL problem encountered calculating normal, using zero." << std::endl;

        tmp_floats[0] = 0;
        tmp_floats[1] = 0;
        tmp_floats[2] = 0;
      }
    }

    tmp_floats[3] = static_cast<float>(v3[0]);
    tmp_floats[4] = static_cast<float>(v3[1]);
    tmp_floats[5] = static_cast<float>(v3[2]);

    tmp_floats[6] = static_cast<float>(v2[0]);
    tmp_floats[7] = static_cast<float>(v2[1]);
    tmp_floats[8] = static_cast<float>(v2[2]);

    tmp_floats[9]  = static_cast<float>(v1[0]);
    tmp_floats[10] = static_cast<float>(v1[1]);
    tmp_floats[11] = static_cast<float>(v1[2]);

    out.write(tmp_floats, 12);
    out << att;
  }
}

}  // un-named

xreg::TriMesh xreg::ReadSTLMesh(const std::string& path, const bool remove_dups)
{
  std::ifstream in(path.c_str(), std::ios::binary);
  if (in.is_open())
  {
    return ReadSTLMesh(in, remove_dups);
  }
  else
  {
    xregThrow("Unable to open STL file for reading!");
  }
}

xreg::TriMesh xreg::ReadSTLMesh(std::istream& in, const bool remove_dups)
{
  StdInputStream xreg_in(in);
  return ReadSTLMesh(xreg_in, remove_dups);
}

xreg::TriMesh xreg::ReadSTLMesh(InputStream& in, const bool remove_dups)
{
  TriMesh mesh;

  std::string tmp_str;

  char tmp_buf[6];
  tmp_buf[5] = 0;

  in.read(reinterpret_cast<Stream::uint8*>(tmp_buf), 5);
  tmp_str = tmp_buf;

  if (tmp_str == "solid")
  {
    // ASCII STL File
    // read the rest of this line to skip to the beginning of the triangles
    in.read_ascii_line();
    mesh = ReadSTLMeshASCII(in);
  }
  else
  {
    // Binary STL File
    in.skip(75);
    mesh = ReadSTLMeshBinary(in);
  }

  if (remove_dups)
  {
    // First duplicate vertices need to be removed and the face indices updated
    mesh.remove_duplicate_vertices();

    // Now remove any duplicate faces
    mesh.remove_duplicate_faces();
  }

  return mesh;
}

void xreg::WriteSTLMesh(const std::string& path, const TriMesh& mesh,
                        const bool ascii, const std::string& ascii_name)
{
  FileOutputStream out(path);
  WriteSTLMesh(out, mesh, ascii, ascii_name);
}

void xreg::WriteSTLMesh(OutputStream& out, const TriMesh& mesh,
                        const bool ascii, const std::string& ascii_name)
{
  if (ascii)
  {
    WriteSTLMeshASCII(out, mesh, ascii_name);
  }
  else
  {
    WriteSTLMeshBinary(out, mesh);
  }
}

void xreg::WriteSTLMesh(std::ostream& out, const TriMesh& mesh,
                        const bool ascii, const std::string& ascii_name)
{
  StdOutputStream out_xreg(out);
  
  if (ascii)
  {
    WriteSTLMeshASCII(out_xreg, mesh, ascii_name);
  }
  else
  {
    WriteSTLMeshBinary(out_xreg, mesh);
  }
}

