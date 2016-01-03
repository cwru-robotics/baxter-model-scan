/*
 * model_pattern 
 *
 * a class to describe a k-means clustering pattern in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include "model_recognition/model_pattern.h"
#include <string>
#include <cmath>

ModelPattern::ModelPattern(std::string name,
                           float r, float g, float b,
                           float l, float w, float h)
{
  pattern_label = name;

  red = r;
  green = g;
  blue = b;

  length = l;
  width = w;
  height = h;

  cluster_label = "none";
}
ModelPattern::~ModelPattern()
{
}

float ModelPattern::EuclidianDistance(std::vector<float> cen_vec)
{
  float distance = 0;

  distance += (red - cen_vec[0]) * (red - cen_vec[0]);
  distance += (green - cen_vec[1]) * (green - cen_vec[1]);
  distance += (blue - cen_vec[2]) * (blue - cen_vec[2]);

  distance += (length - cen_vec[3]) * (length - cen_vec[3]);
  distance += (width - cen_vec[4]) * (width - cen_vec[4]);
  distance += (height - cen_vec[5]) * (height - cen_vec[5]);

  return std::sqrt(distance);
}
