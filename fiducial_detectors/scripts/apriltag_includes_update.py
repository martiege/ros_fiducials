import os
import sys
import glob
import argparse


def create_family_include(namespace, enum, apriltag_include_path):
    output_file_base = """#pragma once

// Standard Library
#include <map>
#include <string>

  """

    apriltag_include_string = """// Apriltag Family includes
extern "C" {
"""

    apriltag_enum_string = """// Apriltag Family Enum
enum class """ + enum + """
{
  undefined,
"""

    apriltag_static_map_string = """// Static Constant Map
static std::map<std::string, """ + enum + """> const """ + enum + """Map
{
"""

    apriltag_family_string_convertion_string = """// String to """ + enum + """
inline """ + enum + """ stringTo""" + enum + """(const std::string& family)
{
  auto it = """ + enum + """Map.find(family);
  if (it == """ + enum + """Map.end())
    return """ + enum + """::undefined;
  else
    return it->second;
}
"""

    apriltag_create_function = """// Create apriltag_family*
inline apriltag_family_t* create""" + enum + """(const """ + enum + """& family)
{
  switch (family)
  {
"""

    apriltag_destroy_function = """// Destroy apriltag_family*
inline void destroy""" + enum + """(apriltag_family_t* tf, const """ + enum + """& family)
{
  switch (family)
  {
"""

    tag_paths_wildcard = os.path.join(apriltag_include_path, "tag*.h")
    tag_files = sorted(glob.glob(tag_paths_wildcard))

    for tag_file in tag_files:
        tag_name = os.path.basename(tag_file)
        apriltag_include_string    += '#include "' + tag_name + '"\n'
        apriltag_enum_string       += 2 * " " + tag_name[:-2] + ",\n"
        apriltag_static_map_string += 2 * " " + '{ "' + tag_name[:-2] + '", ' + enum + '::' + tag_name[:-2] + " }, \n"
        apriltag_create_function   += 2 * " " + "case " + enum + "::" + tag_name[:-2] + ":\n"
        apriltag_create_function   += 4 * " " + "return " + tag_name[:-2] + "_create(); \n"
        apriltag_destroy_function  += 2 * " " + "case " + enum + "::" + tag_name[:-2] + ":\n"
        apriltag_destroy_function  += 4 * " " + tag_name[:-2] + "_destroy(tf); \n"
        apriltag_destroy_function  += 4 * " " + "break; \n"

    apriltag_include_string    += "}\n"
    apriltag_enum_string       += "};\n"
    apriltag_static_map_string += "};\n"
    apriltag_create_function   += 2 * " " + "}\n\n" + 2 * " " + "return nullptr; \n}\n"
    apriltag_destroy_function  += 2 * " " + "}\n}\n"

    result = ""

    result += output_file_base

    result += apriltag_include_string

    result += "\nnamespace " + namespace + "\n{\n\n"

    result += apriltag_enum_string
    result += '\n'
    result += apriltag_static_map_string
    result += '\n'
    result += apriltag_family_string_convertion_string
    result += '\n'
    result += apriltag_create_function
    result += '\n'
    result += apriltag_destroy_function

    result += "\n} // namespace " + namespace + "\n"

    return result

def store_include(path, include_content):
    if os.path.exists(path):
        with open(path) as old:
            if old.read() == include_content:
                return
    with open(path, 'w') as out:
        out.write(include_content)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dynamic creation of ApriltagFamilies based on the apriltag library in pkgconfig")
    parser.add_argument('--namespace', dest="namespace", type=str, help="namespace for the library")
    parser.add_argument('--output_path', dest="output_path", type=str, help="path to system include for apriltag (must be included to find tag files!)")
    parser.add_argument('--apriltag_include_dir', dest="apriltag_include_dir", type=str, help="output path for created file (must be to the include directory!)")
    parser.add_argument('--enum', dest="enum", type=str, help="enumeration name for the library", default="ApriltagFamily")

    args = parser.parse_args()

    if args.output_path.split('/')[-1] == "include":
        print('Creating "apriltag_defined_families.h"')
        print(args.namespace)
        print(args.enum)
        print(args.apriltag_include_dir)
        content = create_family_include(args.namespace, args.enum, args.apriltag_include_dir)
        output_file_path = args.output_path + "/" + args.namespace + "/" + "apriltag_defined_families.h"
        store_include(output_file_path, content)
    else:
        print("output_path must lead to the include directory!")
        sys.exit(1)
