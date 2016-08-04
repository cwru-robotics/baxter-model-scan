/*
 * model_cluster 
 *
 * a class to describe a k-means cluster
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include <ros/ros.h>
#include <string>
#include <vector>

#include "model_recognition/model_cluster.h"

ModelCluster::ModelCluster(std::string label)
{
  cluster_label = label;

  r_centroid = 0;
  b_centroid = 0;
  g_centroid = 0;

  l_centroid = 0;
  w_centroid = 0;
  h_centroid = 0;
}

ModelCluster::~ModelCluster()
{
}

void ModelCluster::initCluster(ModelPattern pattern)
{
  patterns.push_back(pattern);
  r_centroid = pattern.getRed();
  g_centroid = pattern.getGreen();
  b_centroid = pattern.getBlue();

  l_centroid = pattern.getLength();
  w_centroid = pattern.getWidth();
  h_centroid = pattern.getHeight();
}

void ModelCluster::addToCluster(ModelPattern &pattern)
{
  float pat_size = static_cast<float>(patterns.size());

  r_centroid *= pat_size/(pat_size+1);
  r_centroid += ((1/(pat_size+1)) * pattern.getRed());

  g_centroid *= pat_size/(pat_size+1);
  g_centroid += (1/(pat_size+1)) * pattern.getGreen();

  b_centroid *= pat_size/(pat_size+1);
  b_centroid += (1/(pat_size+1)) * pattern.getBlue();

  l_centroid *= pat_size/(pat_size+1);
  l_centroid += (1/(pat_size+1)) * pattern.getLength();

  w_centroid *= pat_size/(pat_size+1);
  w_centroid += (1/(pat_size+1)) * pattern.getWidth();

  h_centroid *= pat_size/(pat_size+1);
  h_centroid += (1/(pat_size+1)) * pattern.getHeight();

  patterns.push_back(pattern);
}

void ModelCluster::removeFromCluster(ModelPattern pattern)
{
  float pat_size = static_cast<float>(patterns.size());

  std::string label = pattern.getLabel();
  for (size_t i = 0; i < patterns.size(); i++)
  {
    if (label == patterns.at(i).getLabel())
      patterns.erase(patterns.begin()+i);
  }

  r_centroid *= pat_size/(pat_size-1);
  r_centroid -= (1/(pat_size-1)) * pattern.getRed();

  g_centroid *= pat_size/(pat_size-1);
  g_centroid -= (1/(pat_size-1)) * pattern.getGreen();

  b_centroid *= pat_size/(pat_size-1);
  b_centroid -= (1/(pat_size-1)) * pattern.getBlue();

  l_centroid *= pat_size/(pat_size-1);
  l_centroid -= (1/(pat_size-1)) * pattern.getLength();

  w_centroid *= pat_size/(pat_size-1);
  w_centroid -= (1/(pat_size-1)) * pattern.getWidth();

  h_centroid *= pat_size/(pat_size-1);
  h_centroid -= (1/(pat_size-1)) * pattern.getHeight();
}

void ModelCluster::printCentroid()
{
  std::cout << cluster_label << " | cR: " << r_centroid << ", cG: " << g_centroid
            << ", cB: " << b_centroid << ", cL: " << l_centroid
            << ", cW: " << w_centroid << ", cH: " << h_centroid << std::endl;
}

void ModelCluster::printMislabelReport()
{
  uint numMislabeled = 0;

  for (size_t i = 0; i < patterns.size(); i++)
  {
    if (patterns.at(i).getLabel().compare(cluster_label) != 0)
      numMislabeled++;
  }

  ROS_INFO("%i patterns have been miscategorized.", numMislabeled);
}

std::vector<float> ModelCluster::getCentroid()
{
  std::vector<float> v;

  v.push_back(r_centroid);
  v.push_back(g_centroid);
  v.push_back(b_centroid);

  v.push_back(l_centroid);
  v.push_back(w_centroid);
  v.push_back(h_centroid);

  return v;
}

