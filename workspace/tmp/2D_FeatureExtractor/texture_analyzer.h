#ifndef TEXTURE_ANALYZER_H_
#define TEXTURE_ANALYZER_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "Describable.h"
#include "typedefs.h"

namespace cv {
class Mat;
}

class ImageAnalyzer: public Describable {
public:
	ImageAnalyzer(); ///< Default constructor
	virtual ~ImageAnalyzer(); ///< Default destructor
	virtual void PerformImageAnalysis(cv::Mat *image_to_analyze,
			cv::Mat *feature_vector_to_place, int pos_feature) = 0;

	virtual void PerformImageAnalysis_oneImage(cv::Mat &image_to_analyze, vector<double>& features)=0;

	virtual int GetNumberOfFeatures() = 0;
	virtual string GetClassName() = 0;
	virtual string GetComments() = 0;
	virtual string GetCurrentParams() = 0;
};

class AbstractCooMatrixTextureAnalyzer: public ImageAnalyzer {
public:

	AbstractCooMatrixTextureAnalyzer(
			AbstractCooMatrixGeneratorPtr glcm_generator,
			int num_windows_height, int num_windows_width,
			vector<FeatureCalculatorPtr> feature_calculators);

	void SetNumWindows(int num_windows_height, int num_windows_width);
	void ChangeFeatureCalculators(
			vector<FeatureCalculatorPtr> feature_calculators);
	virtual ~AbstractCooMatrixTextureAnalyzer();

	virtual void PerformImageAnalysis(cv::Mat *image_to_analyze,
			cv::Mat *feature_vector_to_place, int pos_feature) = 0;

	virtual void PerformImageAnalysis_oneImage(cv::Mat &image_to_analyze, vector<double>& features)=0;

	virtual void SetSeparationWindowsSize(int height, int width);

	virtual void GetSeparationWindowsSize(int *height, int *width);

	virtual int GetNumberOfFeatures() = 0;
	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();
protected:

	virtual void QuantizeGrayMatrix(cv::Mat *image_input_output,
			const int &current_num_levels, const int &desired_num_levels);
	void PrintFeatures(const string &file_to_output, vector<double> *features,
			const string &text_before, const string &text_after);

	AbstractCooMatrixGeneratorPtr coo_matrix_generator_; ///< A pointer to @ref AbstractCooMatrixGenerator
	vector<FeatureCalculatorPtr> feature_calculators_; ///< A vector of pointers to @ref FeatureCalculator
	int num_windows_height_; ///< The number of co-occurrence matrices that will be generated from the input image in the \e y-axis.
	int num_windows_width_; ///< The number of co-occurrence matrices that will be generated from the input image in the \e x-axis.
	int windows_separation_height_; ///< An integer denoting the separation of rows between the starting points of the GLCMs.
	int windows_separation_width_; ///< An integer denoting the separation of columns between the starting points of the GLCMs.
};

class GrayCooMatrixTextureAnalyzer: public AbstractCooMatrixTextureAnalyzer {
public:

	GrayCooMatrixTextureAnalyzer(
			AbstractCooMatrixGeneratorPtr coo_matrix_1c_generator,
			int num_windows_height, int num_windows_width,
			vector<FeatureCalculatorPtr> feature_calculators);

	virtual ~GrayCooMatrixTextureAnalyzer();

	virtual void PerformImageAnalysis(cv::Mat *image_to_analyze,
			cv::Mat *feature_vector_to_place, int pos_feature);

	virtual void PerformImageAnalysis_oneImage(cv::Mat &image_to_analyze, vector<double>& features);

	virtual int GetNumberOfFeatures();
	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();
};

class ColorCooMatrixTextureAnalyzer: public AbstractCooMatrixTextureAnalyzer {
public:

	ColorCooMatrixTextureAnalyzer(
			AbstractCooMatrixGeneratorPtr coo_matrix_3c_generator,
			int num_windows_height, int num_windows_width,
			vector<FeatureCalculatorPtr> feature_calculators);

	virtual ~ColorCooMatrixTextureAnalyzer();
	virtual void PerformImageAnalysis(cv::Mat *image_to_analyze,
			cv::Mat *feature_vector_to_place, int pos_feature);

	virtual void PerformImageAnalysis_oneImage(cv::Mat &image_to_analyze, vector<double>& features);

	virtual int GetNumberOfFeatures();
	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();
};

#endif /* TEXTURE_ANALYZER_H_ */
