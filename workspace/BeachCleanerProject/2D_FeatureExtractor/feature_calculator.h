#ifndef FEATURE_CALCULATOR_H_
#define FEATURE_CALCULATOR_H_
#include "Describable.h"
#include <vector>
#include <string>
namespace cv {
class Mat;
}

class FeatureCalculator: public Describable {
public:
	FeatureCalculator();

	virtual ~FeatureCalculator();
	virtual vector<double> CalculeFeatures(const cv::Mat * data) = 0;
	virtual void CalculeFeatures(const cv::Mat *data, vector<double> *output,
			int start_index) = 0;
	virtual int GetNumberOfFeatures() = 0;
	virtual vector<string> GetFeatureNames() = 0;

	virtual string GetClassName() = 0;
	virtual string GetComments() = 0;
	virtual string GetCurrentParams() = 0;
};

/////////////////////////////// Variance Calculator Class ////////////////////////////////////////
class VarianceCalculator: public FeatureCalculator {
	using FeatureCalculator::GetNumberOfFeatures;

public:
	VarianceCalculator(); ///< Default Constructor
	virtual ~VarianceCalculator(); ///< Default destructor
	virtual vector<double> CalculeFeatures(const cv::Mat * data);
	virtual void CalculeFeatures(const cv::Mat *data, vector<double> *output,
			int start_index);

	virtual int GetNumberOfFeatures();
	virtual vector<string> GetFeatureNames();

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();

private:
	vector<string> feature_names_;
};

/////////////////////////////// Haralick Feature Calculator Class ////////////////////////////////////////
class HaralickFeatureCalculator: public FeatureCalculator {
	using FeatureCalculator::GetNumberOfFeatures;
public:

	HaralickFeatureCalculator(); ///< Default Constructor
	virtual ~HaralickFeatureCalculator(); ///< Default Destructor
	virtual vector<double> CalculeFeatures(const cv::Mat * data);
	virtual void CalculeFeatures(const cv::Mat *data, vector<double> *output,
			int start_index);
	virtual int GetNumberOfFeatures();

	virtual vector<string> GetFeatureNames();

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();

private:
	vector<string> feature_names_;
};

class EneConCorrFeatureCalculator: public FeatureCalculator {
public:
	EneConCorrFeatureCalculator();
	~EneConCorrFeatureCalculator();

	virtual vector<double> CalculeFeatures(const cv::Mat * data);
	virtual void CalculeFeatures(const cv::Mat *data, vector<double> *output,
			int start_index);
	virtual int GetNumberOfFeatures();
	virtual vector<string> GetFeatureNames();

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();

private:
	vector<string> feature_names_;
};

#endif /* FEATURE_CALCULATOR_H_ */
