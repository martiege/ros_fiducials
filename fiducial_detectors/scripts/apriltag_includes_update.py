import sys 
import glob 
import pkgconfig 
import argparse 

def create_family_include(output_path, namespace, enum): 
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

  output_file_path = output_path + "/" + namespace + "/" + "apriltag_defined_families.h"

  apriltag_include_path = pkgconfig.cflags('apriltag')[2:] + "/apriltag/"
  tag_files = sorted(glob.glob(apriltag_include_path + "tag*.h"))

  for tag_file in tag_files: 
    tag_name = tag_file[len(apriltag_include_path):]
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

  with open(output_file_path, 'w') as out: 
    out.write(output_file_base) 

    out.write(apriltag_include_string)

    out.write("\nnamespace " + namespace + "\n{\n\n")
    
    out.write(apriltag_enum_string)
    out.write('\n')
    out.write(apriltag_static_map_string)
    out.write('\n')
    out.write(apriltag_family_string_convertion_string)
    out.write('\n')
    out.write(apriltag_create_function)
    out.write('\n')
    out.write(apriltag_destroy_function)

    out.write("\n} // namespace " + namespace + "\n")

if __name__ == "__main__": 
  parser = argparse.ArgumentParser(description="Dynamic creation of ApriltagFamilies based on the apriltag library in pkgconfig") 
  parser.add_argument('--namespace', dest="namespace", type=str, help="namespace for the library")
  parser.add_argument('--output_path', dest="output_path", type=str, help="output path for created file (must be to the include directory!)")
  parser.add_argument('--enum', dest="enum", type=str, help="enumeration name for the library", default="ApriltagFamily") 

  args = parser.parse_args()

  if args.output_path.split('/')[-1] == "include": 
    create_family_include(args.output_path, args.namespace, args.enum)
  else: 
    print("output_path must lead to the include directory!")
    sys.exit(1)