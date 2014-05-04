
#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <boost/shared_ptr.hpp>

class ImageAnalyzer;
class AbstractCooMatrixTextureAnalyzer;
class GrayCooMatrixTextureAnalyzer;
class ColorCooMatrixTextureAnalyzer;

class FeatureCalculator;
class AbstractCooMatrixGenerator;
class FeaturesFormatManager;


typedef boost::shared_ptr<ImageAnalyzer> ImageAnalyzerPtr;
typedef boost::shared_ptr<FeaturesFormatManager> FeaturesFormatManagerPtr;
typedef boost::shared_ptr<AbstractCooMatrixTextureAnalyzer> AbstractCooMatrixTextureAnalyzerPtr;

typedef boost::shared_ptr<GrayCooMatrixTextureAnalyzer> GrayCooMatrixTextureAnalyzerPtr;
typedef boost::shared_ptr<ColorCooMatrixTextureAnalyzer> ColorCooMatrixTextureAnalyzerPtr;

typedef boost::shared_ptr<FeatureCalculator> FeatureCalculatorPtr;
typedef boost::shared_ptr<AbstractCooMatrixGenerator> AbstractCooMatrixGeneratorPtr;

#endif /* TYPEDEFS_H_ */
