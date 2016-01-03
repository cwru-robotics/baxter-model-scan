/*
 * model_pattern_set
 *
 * a class to describe a k-means clustering pattern set in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#ifndef MODEL_PATTERN_SET
#define MODEL_PATTERN_SET

#include <string>
#include <vector>

#include "model_recognition/model_pattern.h"

class ModelPatternSet
{
public:
  ModelPatternSet();
  virtual ~ModelPatternSet();

private:
  std::vector<ModelPattern> pattern_set;

public:
  void pushBackPattern(ModelPattern mp);
  ModelPattern & getPattern(uint position);

  ModelPattern getPatternCopy(uint position);
  void scaleFeatures();
  uint getSize(){ return pattern_set.size(); }


};

#endif  // MODEL_PATTERN_SET
