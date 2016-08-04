/*
 * model_pattern 
 *
 * a class to describe a k-means clustering pattern in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#ifndef MODEL_PATTERN_H
#define MODEL_PATTERN_H

#include <string>
#include <vector>

class ModelPattern
{
public:
  ModelPattern(std::string name,
               float r, float g, float b,
               float l, float w, float h);
  virtual ~ModelPattern();

private:
  std::string pattern_label;

  std::string cluster_label;

  float red;
  float green;
  float blue;

  float length;
  float width;
  float height;

public:
  // Getters
  std::string getLabel() { return pattern_label; }

  std::string getClusterLabel() { return cluster_label; }

  float getRed() { return red; }
  float getGreen() { return green; }
  float getBlue() { return blue; }

  float getLength() { return length; }
  float getWidth() { return width; }
  float getHeight() { return height; }

  void setRed(float r) { red = r; }
  void setGreen(float g) { green = g; }
  void setBlue(float b) { blue = b; }

  void setLength(float l) { length = l; }
  void setWidth(float w) { width = w; }
  void setHeight(float h) { height = h; }

  // Setters
  void setClusterLabel(std::string label) { cluster_label = label; }

  float EuclidianDistance(std::vector<float> cen_vec);
};

#endif  // MODEL_PATTERN_H
