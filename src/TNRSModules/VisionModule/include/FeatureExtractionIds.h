/**
 * @file VisionModule/VisionModuleIds.h
 *
 * This file declares the Enumeration Ids for all the feature extraction
 * modules under vision module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Sep 2017
 */

#pragma once

/**
 * Enumeration for the classes inheriting FeatureExtraction class
 * 
 * @enum FeatureExtractionIds
 */
enum class FeatureExtractionIds : unsigned int
{
  segmentation,
  field,
  robot,
  goal,
  ball,
  lines,
  count
};
