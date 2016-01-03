/*
 * model_pattern_set
 *
 * a class to describe a k-means clustering pattern set in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include "model_recognition/model_pattern_set.h"

ModelPatternSet::ModelPatternSet()
{
}

ModelPatternSet::~ModelPatternSet()
{
}

void ModelPatternSet::pushBackPattern(ModelPattern mp)
{
  pattern_set.push_back(mp);
}

ModelPattern& ModelPatternSet::getPattern(uint position)
{
  return pattern_set.at(position);
}

ModelPattern ModelPatternSet::getPatternCopy(uint position)
{
  return pattern_set.at(position);
}

void ModelPatternSet::scaleFeatures()
{
  float max_red = -1.0;
  float max_blu = -1.0;
  float max_grn = -1.0;
  float max_l = -1.0;
  float max_w = -1.0;
  float max_h = -1.0;

  float min_red = 400.0;
  float min_blu = 400.0;
  float min_grn = 400.0;
  float min_l = 999999.0;
  float min_w = 999999.0;
  float min_h = 999999.0;

  for (size_t i = 0; i < pattern_set.size(); i++)
  {
    // FIND MAXIMUMS
    if (pattern_set.at(i).getRed() > max_red)
      max_red = pattern_set.at(i).getRed();

    if (pattern_set.at(i).getGreen() > max_grn)
      max_grn = pattern_set.at(i).getGreen();

    if (pattern_set.at(i).getBlue() > max_blu)
      max_blu = pattern_set.at(i).getBlue();

    if (pattern_set.at(i).getLength() > max_l)
      max_l = pattern_set.at(i).getLength();

    if (pattern_set.at(i).getWidth() > max_w)
      max_w = pattern_set.at(i).getWidth();

    if (pattern_set.at(i).getHeight() > max_h)
      max_h = pattern_set.at(i).getHeight();

    // FIND MINIMUMS
    if (pattern_set.at(i).getRed() < min_red)
      min_red = pattern_set.at(i).getRed();

    if (pattern_set.at(i).getGreen() < min_grn)
      min_grn = pattern_set.at(i).getGreen();

    if (pattern_set.at(i).getBlue() < min_blu)
      min_blu = pattern_set.at(i).getBlue();

    if (pattern_set.at(i).getLength() < min_l)
      min_l = pattern_set.at(i).getLength();

    if (pattern_set.at(i).getWidth() < min_w)
      min_w = pattern_set.at(i).getWidth();

    if (pattern_set.at(i).getHeight() < min_h)
      min_h = pattern_set.at(i).getHeight();
  }

  // SCALE FEATURES

  for (size_t i = 0; i < pattern_set.size(); i++)
  {
    pattern_set.at(i).setRed((pattern_set.at(i).getRed() - min_red)/(max_red - min_red));
    pattern_set.at(i).setGreen((pattern_set.at(i).getGreen() - min_grn)/(max_grn - min_grn));
    pattern_set.at(i).setBlue((pattern_set.at(i).getBlue() - min_blu)/(max_blu - min_blu));

    pattern_set.at(i).setLength((pattern_set.at(i).getLength() - min_l)/(max_l - min_l));
    pattern_set.at(i).setWidth((pattern_set.at(i).getWidth() - min_w)/(max_w - min_w));
    pattern_set.at(i).setHeight((pattern_set.at(i).getHeight() - min_h)/(max_h - min_h));
  }
}
