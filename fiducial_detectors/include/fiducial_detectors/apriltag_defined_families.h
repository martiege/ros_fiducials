#pragma once 

// Standard Library 
#include <map> 
#include <string> 

  // Apriltag Family includes 
extern "C" {
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

namespace fiducial_detectors
{

// Apriltag Family Enum
enum class ApriltagFamily
{
  undefined, 
  tag16h5,
  tag25h9,
  tag36h11,
  tagCircle21h7,
  tagCircle49h12,
  tagCustom48h12,
  tagStandard41h12,
  tagStandard52h13,
};

// Static Constant Map
static std::map<std::string, ApriltagFamily> const ApriltagFamilyMap
{
  { "tag16h5", ApriltagFamily::tag16h5 }, 
  { "tag25h9", ApriltagFamily::tag25h9 }, 
  { "tag36h11", ApriltagFamily::tag36h11 }, 
  { "tagCircle21h7", ApriltagFamily::tagCircle21h7 }, 
  { "tagCircle49h12", ApriltagFamily::tagCircle49h12 }, 
  { "tagCustom48h12", ApriltagFamily::tagCustom48h12 }, 
  { "tagStandard41h12", ApriltagFamily::tagStandard41h12 }, 
  { "tagStandard52h13", ApriltagFamily::tagStandard52h13 }, 
};

// String to ApriltagFamily 
inline ApriltagFamily stringToApriltagFamily(const std::string& family)
{
  auto it = ApriltagFamilyMap.find(family); 
  if (it == ApriltagFamilyMap.end())
    return ApriltagFamily::undefined; 
  else 
    return it->second; 
}

// Create apriltag_family*
inline apriltag_family_t* createApriltagFamily(const ApriltagFamily& family) 
{
  switch (family)
  {
  case ApriltagFamily::tag16h5:
    return tag16h5_create(); 
  case ApriltagFamily::tag25h9:
    return tag25h9_create(); 
  case ApriltagFamily::tag36h11:
    return tag36h11_create(); 
  case ApriltagFamily::tagCircle21h7:
    return tagCircle21h7_create(); 
  case ApriltagFamily::tagCircle49h12:
    return tagCircle49h12_create(); 
  case ApriltagFamily::tagCustom48h12:
    return tagCustom48h12_create(); 
  case ApriltagFamily::tagStandard41h12:
    return tagStandard41h12_create(); 
  case ApriltagFamily::tagStandard52h13:
    return tagStandard52h13_create(); 
  }

  return nullptr; 
}

// Destroy apriltag_family*
inline void destroyApriltagFamily(apriltag_family_t* tf, const ApriltagFamily& family) 
{
  switch (family)
  {
  case ApriltagFamily::tag16h5:
    tag16h5_destroy(tf); 
    break; 
  case ApriltagFamily::tag25h9:
    tag25h9_destroy(tf); 
    break; 
  case ApriltagFamily::tag36h11:
    tag36h11_destroy(tf); 
    break; 
  case ApriltagFamily::tagCircle21h7:
    tagCircle21h7_destroy(tf); 
    break; 
  case ApriltagFamily::tagCircle49h12:
    tagCircle49h12_destroy(tf); 
    break; 
  case ApriltagFamily::tagCustom48h12:
    tagCustom48h12_destroy(tf); 
    break; 
  case ApriltagFamily::tagStandard41h12:
    tagStandard41h12_destroy(tf); 
    break; 
  case ApriltagFamily::tagStandard52h13:
    tagStandard52h13_destroy(tf); 
    break; 
  }
}

} // namespace fiducial_detectors
