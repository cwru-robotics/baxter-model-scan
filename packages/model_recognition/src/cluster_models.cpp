/*
 * cluster_models
 *
 * a ros node to perform k-means clustering in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <vector>

#include "model_recognition/model_cluster.h"

#include "model_recognition/model_pattern_set.h"

typedef std::vector<ModelCluster> ModelClusterSet;

bool g_debug = false;

uint N_CLUSTERS = 3;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_models");
  ros::NodeHandle nh;

  ModelPatternSet pattern_set;
  ModelClusterSet cluster_set;

  std::fstream feature_info("/home/luc/indigo/feature_info.dat", std::ios_base::in);

  std::string read_in;

  uint count_iter = 0;
  uint n_features = 7;

  std::string p_name;
  float feat [n_features - 1];

  while (feature_info >> read_in)
  {
    if (count_iter % n_features == 0)
    {
      // PATTERN NAME
      p_name = read_in;
      // std::cout << "name: " << p_name << std::endl;
    }
    else if (count_iter % n_features == 1)
    {
      feat[0] = std::stof(read_in);
      // std::cout << "red: " << feat[0] << std::endl;
    }
    else if (count_iter % n_features == 2)
    {
      // PATTERN GREEN
      feat[1] = std::stof(read_in);
      // std::cout << "grn: " << feat[1] << std::endl;
    }
    else if (count_iter % n_features == 3)
    {
      // PATTERN BLUE
      feat[2] = std::stof(read_in);
      // std::cout << "blu: " << feat[2] << std::endl;
    }
    else if (count_iter % n_features == 4)
    {
      // PATTERN LENGTH
      feat[3] = std::stof(read_in);
      // std::cout << "l: " << feat[3] << std::endl;
    }
    else if (count_iter % n_features == 5)
    {
      // PATTERN WIDTH
      feat[4] = std::stof(read_in);
      // std::cout << "w: " << feat[4] << std::endl;
    }
    else if (count_iter % n_features == 6)
    {
      // PATTERN HEIGHT
      feat[5] = std::stof(read_in);
      // std::cout << "h: " << feat[5] << std::endl;

      // END READ LINE
      ModelPattern pattern(p_name,
                          feat[0], feat[1], feat[2],
                          feat[3], feat[4], feat[5]);

      pattern_set.pushBackPattern(pattern);
    }
    count_iter++;
  }

  ROS_INFO("Got raw data.");
  if (g_debug)
  {
    for (uint i = 0; i < pattern_set.getSize(); i++)
    {
      ModelPattern p = pattern_set.getPattern(i);
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getGreen()
                << " " <<p.getBlue() << " " << p.getLength() << " "
                << p.getWidth() << " " <<p.getHeight() << std::endl;
    }
  }
  ROS_INFO("Time to pre-process data.  Scaling features.");

  pattern_set.scaleFeatures();

  ROS_INFO("Features have been scaled.");

  if (g_debug)
  {
    for (uint i = 0; i < pattern_set.getSize(); i++)
    {
      ModelPattern p = pattern_set.getPattern(i);
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getGreen()
                << " " <<p.getBlue() << " " << p.getLength() << " "
                << p.getWidth() << " " <<p.getHeight() << std::endl;
    }
  }

  ROS_INFO("Assigning clusters to contain first pattern of each model");
  // gonna cheat a little bit.  initialize each cluster to the first
  // model scan data of each data set.  for our purposes, this
  // simplifies the clustering process and will allow for the easy
  // introduction of new data.

  std::string label = "";

  for (size_t i = 0; i < pattern_set.getSize(); i++)
  {
    if (pattern_set.getPattern(i).getLabel() != label)
    {
      label = pattern_set.getPattern(i).getLabel();
      ModelCluster m(label);
      pattern_set.getPattern(i).setClusterLabel(m.getLabel());

      m.initCluster(pattern_set.getPattern(i));
      cluster_set.push_back(m);
    }
  }

  if (g_debug)
  {
    for (size_t i = 0; i < cluster_set.size(); i++)
    {
      cluster_set.at(i).printCentroid();
    }
  }

  ROS_INFO("Clusters initialized!  Ready for pattern processing.");

  uint nChanges = 0;

  if (g_debug)
  {
    for (size_t i = 0; i < cluster_set.size(); i++)
    {
      cluster_set.at(i).printCentroid();
      std::cout << std::endl;
    }
  }

  for (size_t p = 0; p < pattern_set.getSize(); p++)
  {
    if (pattern_set.getPattern(p).getClusterLabel().compare("none") == 0)
    {
      uint rand_clust = rand() % 3;

      pattern_set.getPattern(p).setClusterLabel(cluster_set.at(rand_clust).getLabel());
      cluster_set.at(rand_clust).addToCluster(pattern_set.getPattern(p));
    }
  }

  double time_begin = ros::Time::now().toSec();

  ROS_INFO("Doing clustering.");
  uint reassignments = 0;
  do
  {
    nChanges = 0;
    for (size_t c = 0; c < cluster_set.size(); c++)
    {
      for (size_t p = 0; p < pattern_set.getSize(); p++)
      {
        // Find the cluster index in which the current pattern is currently assigned
        uint asgn_clust_i = -1;
        for (size_t i = 0; i < cluster_set.size(); i++)
        {
          if (pattern_set.getPattern(p).getClusterLabel().compare(cluster_set.at(i).getLabel()) == 0)
          {
            asgn_clust_i = i;
          }
        }
        // If the new cluster distance is less than the old cluster distance...
        if (pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(c).getCentroid()) <
            pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(asgn_clust_i).getCentroid()))
        {
          // The pattern should be removed from asgn_clust_i and assigned to c
          cluster_set.at(asgn_clust_i).removeFromCluster(pattern_set.getPattern(p));
          pattern_set.getPattern(p).setClusterLabel(cluster_set.at(c).getLabel());
          cluster_set.at(c).addToCluster(pattern_set.getPattern(p));

          nChanges += 2;
          reassignments++;
        }
      }
    }
  }
  while (nChanges != 0);

  double clustering_time = ros::Time::now().toSec() - time_begin;

  std::cout << std::endl;
  ROS_INFO("Done clustering! %i reassignments made in %f seconds.", reassignments, clustering_time);

  ROS_INFO("report: ");
  for (size_t i = 0; i < cluster_set.size(); i++)
  {
    cluster_set.at(i).printCentroid();
    cluster_set.at(i).printMislabelReport();
    
    std::cout << std::endl;
  }

  // Now we'll try to do 'recognition'

  // Make new patterns
  ModelPattern mystery1("mystery", 210, 210, 210, .07, .07, .14);  // 'natty-like' object
  ModelPattern mystery2("mystery", 160, 38, 14, .07, .075, .09);  // 'apple-like' object
  ModelPattern mystery3("mystery", 43, 55, 120, .10, .04, .17);  // 'kraft-like' object

  // scale them
  ModelPatternSet mystery_set;
  mystery_set.pushBackPattern(mystery1);
  mystery_set.pushBackPattern(mystery2);
  mystery_set.pushBackPattern(mystery3);

  mystery_set.scaleFeatures();

  // Identify mystery 1
  uint mystery1_set = -1;
  float mystery1_dist = 999999;
  for (size_t i = 0; i < cluster_set.size(); i++)
  {
    if (mystery_set.getPattern(0).EuclidianDistance(cluster_set.at(i).getCentroid()) < mystery1_dist)
    {
      mystery1_dist = mystery_set.getPattern(0).EuclidianDistance(cluster_set.at(i).getCentroid());
      mystery1_set = i;
    }
  }

  std::cout << "mystery1 is: " << cluster_set.at(mystery1_set).getLabel() << std::endl;

  // Identify mystery 2
  uint mystery2_set = -1;
  float mystery2_dist = 999999;
  for (size_t i = 0; i < cluster_set.size(); i++)
  {
    if (mystery_set.getPattern(1).EuclidianDistance(cluster_set.at(i).getCentroid()) < mystery2_dist)
    {
      mystery2_dist = mystery_set.getPattern(1).EuclidianDistance(cluster_set.at(i).getCentroid());
      mystery2_set = i;
    }
  }

  std::cout << "mystery2 is: " << cluster_set.at(mystery2_set).getLabel() << std::endl;

  // Identify mystery 3
  uint mystery3_set = -1;
  float mystery3_dist = 999999;
  for (size_t i = 0; i < cluster_set.size(); i++)
  {
    if (mystery_set.getPattern(2).EuclidianDistance(cluster_set.at(i).getCentroid()) < mystery3_dist)
    {
      mystery3_dist = mystery_set.getPattern(2).EuclidianDistance(cluster_set.at(i).getCentroid());
      mystery3_set = i;
    }
  }

  std::cout << "mystery3 is: " << cluster_set.at(mystery3_set).getLabel() << std::endl;
}
