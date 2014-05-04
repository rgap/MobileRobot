#ifndef FEATURE_FORMATTER_H_
#define FEATURE_FORMATTER_H_

#include <string>
#include <vector>


#include <opencv2/opencv.hpp>

class FeaturesFormatManager {
public:
	FeaturesFormatManager();
	virtual ~FeaturesFormatManager();
	virtual void Print(const string &output_file_name,
			const vector<string> &input_names,
			const vector<string> &input_classes,
			const cv::Mat & input_features, int mode = 0) throw (exception) = 0;
	virtual void Retrieve(const string &input_file_name,
			vector<string> *input_names, vector<string> *input_classes,
			cv::Mat *input_features, vector<string> *distinct_classes)
			throw (exception) = 0;
	int GetNumberOfFeatures();
	int GetNumberOfInputs();

	void SetNumberOfFeatures(int number_of_features);
	void SetNumberOfInputs(int number_of_inputs);
private:
	int number_of_features_;
	int number_of_inputs_;
};

///////////////////////////////////////////// SUBCLASES


class PredefinedFormatManager: public FeaturesFormatManager {
	using FeaturesFormatManager::GetNumberOfFeatures;
	using FeaturesFormatManager::GetNumberOfInputs;
public:
	PredefinedFormatManager();
	virtual ~PredefinedFormatManager();
	virtual void Print(const string &output_file_name,
			const vector<string> &input_names,
			const vector<string> &input_classes,
			const cv::Mat & input_features, int mode = 0) throw (exception);
	virtual void Retrieve(const string &input_file_name,
			vector<string> *input_names, vector<string> *input_classes,
			cv::Mat *input_features, vector<string> *distinct_classes)
			throw (exception);
private:
	void ExtractNumInputsAndFeatures(const string file_name,
			vector<string> *distinct_classes) throw (exception);
	void GenerateConfFile(const string &training_feature_vec,
			const string& conf_file_name) throw (exception);
};

class WekaFormatManager : public FeaturesFormatManager{
public:
	WekaFormatManager();
	~WekaFormatManager();
	virtual void Print(const std::string &output_file_name, const std::vector<std::string> &input_names, const std::vector<std::string> &input_classes,
								const cv::Mat & input_features, int mode = 0) throw(std::exception);
	virtual void Retrieve(const std::string &input_file_name , std::vector<std::string> *input_names,	std::vector<std::string> *input_classes,
								cv::Mat *input_features, std::vector<std::string> *distinct_classes ) throw(std::exception);
	virtual std::string vector_extension();
private:
	void ToLowerCase(std::string *token);
};

#endif /* FEATURE_FORMATTER_H_ */
