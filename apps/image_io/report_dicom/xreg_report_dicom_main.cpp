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

// Report some metadata from a DICOM series
// Derived from this example: http://www.itk.org/Doxygen/html/Examples_2IO_2DicomImageReadPrintTags_8cxx-example.html

#include <itkImage.h>
#include <itkGDCMImageIO.h>
#include <itkMetaDataObject.h>
#include <itkImageFileReader.h>
#include <itkDirectory.h>

#include <set>
#include <string>

#include "xregProgOptUtils.h"
#include "xregFilesystemUtils.h"
#include "xregDICOMUtils.h"

int main( int argc, char* argv[] )
{
  using namespace xreg;

  const int kEXIT_VAL_SUCCESS    = 0;
  const int kEXIT_VAL_BAD_USE    = 1;
  const int kEXIT_VAL_EMPTY_DIR  = 2;
  const int kEXIT_VAL_IO_FAILURE = 3;

  const unsigned int Dimension  = 3;

  using PixelType = signed short;

  using ImageType = itk::Image<PixelType, Dimension>;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Reports metadata for a DICOM series directory or a single DICOM image. "
              "To limit the output to a specific set of flags, enter them as additional "
              "positional arguments.");
  po.set_arg_usage("<Input DICOM Directory or DICOM File Path> [[[tag 1] tag2] ... tag N]");

  po.add("skip-full-print", 's', ProgOpts::kSTORE_TRUE, "skip-full-print",
         "Skip the raw printout of all DICOM fields.")
    << false;

  po.add("print-xreg-fields", 'x', ProgOpts::kSTORE_TRUE, "print-xreg-fields",
         "Print basic DICOM fields as stored by the DICOMFIleBasicFields struct.")
    << false;

  po.set_min_num_pos_args(1);

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_BAD_USE;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  const bool skip_full_print = po.get("skip-full-print");

  const bool print_xreg_fields = po.get("print-xreg-fields");

  using TagSetType = std::set<std::string>;
  TagSetType in_tags(po.pos_args().begin() + 1, po.pos_args().end());
  const bool limit_tags = !in_tags.empty();

  std::string img_path = po.pos_args()[0];

  if (!skip_full_print)
  {
    // First check for a directory as source input, if true then assume we are
    // dealing with DICOM
    itk::Directory::Pointer src_dir = itk::Directory::New();
    if (src_dir->Load(img_path.c_str()))
    {
      int good_file_idx = -1;

      const int num_files = src_dir->GetNumberOfFiles();

      for (int file_idx = 0; file_idx < num_files; ++file_idx)
      {
        if (src_dir->GetFile(file_idx)[0] != '.')
        {
          good_file_idx = file_idx;
          break;
        }
      }

      if (good_file_idx >= 0)
      {
        img_path += kFILE_SEPS[0];
        img_path += src_dir->GetFile(good_file_idx);
      }
      else
      {
        std::cerr << "ERROR: could not find a file to read!" << std::endl;
        return kEXIT_VAL_EMPTY_DIR;
      }
    }

    using ImageIOType = itk::GDCMImageIO;

    ImageIOType::Pointer dcm_io = ImageIOType::New();

    using ReaderType = itk::ImageFileReader<ImageType>;
    ReaderType::Pointer reader = ReaderType::New();
    reader->SetImageIO(dcm_io);
    reader->SetFileName(img_path);

    try
    {
      reader->Update();
    }
    catch (itk::ExceptionObject& e)
    {
      std::cerr << "ERROR: reading file:\n" << e << std::endl;
      return kEXIT_VAL_IO_FAILURE;
    }

    using DictionaryType = itk::MetaDataDictionary;

    const DictionaryType& dict = dcm_io->GetMetaDataDictionary();

    DictionaryType::ConstIterator dict_end_it = dict.End();

    for (DictionaryType::ConstIterator dict_it = dict.Begin(); dict_it != dict_end_it; ++dict_it)
    {
      using MetaDataStringType = itk::MetaDataObject<std::string>;

      MetaDataStringType::Pointer entry = dynamic_cast<MetaDataStringType*>(dict_it->second.GetPointer());

      if (entry)
      {
        std::string tag_key = dict_it->first;

        // only print out if no tags have been specified, or this tag is a specified tag
        if (!limit_tags || (in_tags.find(tag_key)) != in_tags.end())
        {
          std::string label_id;

          const bool tag_found = itk::GDCMImageIO::GetLabelFromTag(tag_key, label_id);

          std::cout << "(" << tag_key << ") " << (tag_found ? label_id.c_str() : "Unknown") << " = " << entry->GetMetaDataObjectValue() << std::endl;
        }
      }
    }
  }

  if (print_xreg_fields)
  {
    std::cout << "--------- xreg Basic DICOM Fields ---------" << std::endl;

    PrintDICOMFileBasicFields(ReadDICOMFileBasicFields(img_path), std::cout);
  }

  return kEXIT_VAL_SUCCESS;
}
