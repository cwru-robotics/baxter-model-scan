/*
 * model_cluster 
 *
 * a class to describe a k-means cluster
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#ifndef MODEL_CLUSTER_H
#define MODEL_CLUSTER_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "model_recognition/model_pattern.h"

class ModelCluster
{
public:
  ModelCluster(std::string label);
  virtual ~ModelCluster();

private:
  std::vector<ModelPattern> patterns;

  std::string cluster_label;

  float r_centroid;
  float g_centroid;
  float b_centroid;

  float l_centroid;
  float w_centroid;
  float h_centroid;

public:
  void addToCluster(ModelPattern &pattern);
  void removeFromCluster(ModelPattern pattern);
  void initCluster(ModelPattern pattern);

  std::vector<ModelPattern> & getPatternVec() { return patterns; }

  std::vector<float> getCentroid();

  std::string getLabel() { return cluster_label; }

  void printCentroid();
  void printMislabelReport();
};

#endif  // MODEL_CLUSTER_H
