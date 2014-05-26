#include "feature_calculator.h"
#include "opencv2/opencv.hpp"
#include <sstream>

FeatureCalculator::FeatureCalculator() {
	// TODO Auto-generated constructor stub
}

FeatureCalculator::~FeatureCalculator() {
	// TODO Auto-generated destructor stub
}

///////////////////////////////////////// Variance Calculator Class /////////////////////////////////


VarianceCalculator::VarianceCalculator() {
	feature_names_.push_back("Variance");
}

VarianceCalculator::~VarianceCalculator() {
}

vector<double> VarianceCalculator::CalculeFeatures(const cv::Mat* data) {
	return vector<double>();
}

void VarianceCalculator::CalculeFeatures(const cv::Mat* data,
		vector<double>* output, int start_index) {

}

int VarianceCalculator::GetNumberOfFeatures() {
	return 1;
}

vector<string> VarianceCalculator::GetFeatureNames() {
	return feature_names_;
}

//Describable Class implementation

string VarianceCalculator::GetClassName() {
	return "VarianceCalculator";
}

string VarianceCalculator::GetComments() {
	return "It calculates just one feature";
}

string VarianceCalculator::GetCurrentParams() {
	stringstream streamer;
	streamer << "It calculates the following features:" << endl;
	for (int feature_ind = 0; feature_ind < (int) feature_names_.size(); ++feature_ind) {
		streamer << " - " << feature_names_[feature_ind] << endl;
	}
	return streamer.str();
}

/////////////////////////////////////////// Haralick Calculator Class ///////////////////////////////


HaralickFeatureCalculator::HaralickFeatureCalculator() {
	feature_names_.push_back("Angular Second Moment");
	feature_names_.push_back("Contrast");
	feature_names_.push_back("Correlation");
	feature_names_.push_back("Sum of Squares: Variance");
	feature_names_.push_back("Inverse Different Moment");
	feature_names_.push_back("Sum Average");
	feature_names_.push_back("Sum Variance");
	feature_names_.push_back("Sum Entropy");
	feature_names_.push_back("Entropy");
	feature_names_.push_back("Difference Variance");
	feature_names_.push_back("Difference Entropy");
	feature_names_.push_back("Information Measures of Correlation 1");
	feature_names_.push_back("Information Measures of Correlation 2");
}

HaralickFeatureCalculator::~HaralickFeatureCalculator() {
}

vector<double> HaralickFeatureCalculator::CalculeFeatures(
		const cv::Mat* data) {
	vector<double> haralick_features(13, 0.0);
	CalculeFeatures(data, &haralick_features, 0);
	return haralick_features;
}

void HaralickFeatureCalculator::CalculeFeatures(const cv::Mat* data,
		vector<double> * output, int start_index) {
	int cols = data->cols;
	int rows = data->rows;
	const double EPS = 1E-9;

	const double * glcm_row;

	double * p_x = new double[rows];
	double * p_y = new double[cols];
	double * p_xplusy = new double[2 * rows - 1];
	double * p_xminusy = new double[rows];

	// initializing the vectors;
	for (int i = 0; i < rows; i++)
		p_x[i] = p_y[i] = p_xminusy[i] = 0;
	for (int i = 0; i < 2 * rows - 1; i++)
		p_xplusy[i] = 0;

	double mean = 0, mean_x = 0, mean_y = 0, variance_x = 0, variance_y = 0;
	double hx = 0, hy = 0, hxy = 0, hxy1 = 0, hxy2 = 0;
	// filling the vectors;
	for (int i = 0; i < rows; i++) {
		glcm_row = data->ptr<double> (i);
		for (int j = 0; j < cols; j++) {
			p_x[i] += glcm_row[j];
			p_y[j] += glcm_row[j];
			p_xplusy[i + j] += glcm_row[j];
			p_xminusy[abs(i - j)] += glcm_row[j];
			mean_x += ((i + 1) * glcm_row[j]);
			mean_y += ((j + 1) * glcm_row[j]);
			mean += glcm_row[j];

		}
	}
	mean /= (cols * rows);

	double contrast = 0, ang2mom = 0, correlation = 0, sum_of_squares = 0,
			inver_diff_mom = 0, entropy = 0;
	double sum_average = 0, sum_entropy = 0, sum_variance = 0, diff_variance =
			0, diff_entropy = 0;
	double info_meas_corr1 = 0, info_meas_corr2 = 0;
	for (int i = 0; i < rows; i++) {
		glcm_row = data->ptr<double> (i);
		for (int j = 0; j < cols; j++) {
			ang2mom += (glcm_row[j] * glcm_row[j]);
			contrast += ((i - j) * (i - j) * glcm_row[j]);
			variance_x += (glcm_row[j] * (i + 1 - mean_x) * (i + 1 - mean_x));
			variance_y += (glcm_row[j] * (j + 1 - mean_y) * (j + 1 - mean_y));
			correlation += ((i + 1) * (j + 1) * glcm_row[j]);
			sum_of_squares += (glcm_row[j] * (i + 1 - mean) * (i + 1 - mean));
			inver_diff_mom += (glcm_row[j] / (1 + (i - j) * (i - j)));
			//cout << sum_of_squares << " " << i << " " << mean << endl;
			// to calculate Information Measures of Correlation
			hxy1 -= (glcm_row[j] * log(p_x[i] * p_y[j] + EPS));
			hxy2 -= (p_x[i] * p_y[j] * log(p_x[i] * p_y[j] + EPS));
			hxy -= (glcm_row[j] * log(glcm_row[j] + EPS));
		}
		diff_variance += (i * i * (p_xminusy[i]));
		diff_entropy -= (p_xminusy[i] * log(p_xminusy[i] + EPS));

		//to calculate Information Measures of Correlation
		hx -= (p_x[i] * log(p_x[i] + EPS));
		hy -= (p_y[i] * log(p_y[i] + EPS));
	}
	correlation = (correlation - (mean_x * mean_y)) / sqrt(
			variance_x * variance_y);
	entropy = hxy;
	info_meas_corr1 = (hxy - hxy1) / max(hx, hy);
	info_meas_corr2 = sqrt(1 - exp(-2.0 * (hxy2 - hxy)));

	for (int i = 0; i < 2 * rows - 1; i++) {
		sum_average += ((i + 2) * p_xplusy[i]);
		sum_entropy -= (p_xplusy[i] * log(p_xplusy[i] + EPS));
	}
	for (int i = 0; i < 2 * rows - 1; i++) {
		sum_variance += ((i + 2 - sum_entropy) * (i + 2 - sum_entropy)
				* p_xplusy[i]);
	}
	(*output)[start_index++] = ang2mom;
	(*output)[start_index++] = contrast;
	(*output)[start_index++] = correlation;
	(*output)[start_index++] = sum_of_squares;
	(*output)[start_index++] = inver_diff_mom;
	(*output)[start_index++] = sum_average;
	(*output)[start_index++] = sum_variance;
	(*output)[start_index++] = sum_entropy;
	(*output)[start_index++] = entropy;
	(*output)[start_index++] = diff_variance;
	(*output)[start_index++] = diff_entropy;
	(*output)[start_index++] = info_meas_corr1;
	(*output)[start_index++] = info_meas_corr2;
}

vector<string> HaralickFeatureCalculator::GetFeatureNames() {
	return feature_names_;
}

int HaralickFeatureCalculator::GetNumberOfFeatures() {
	return 13;
}

//Describable Class implementation
string HaralickFeatureCalculator::GetClassName() {
	return "HaralickFeatureCalculator";
}

string HaralickFeatureCalculator::GetComments() {
	return "It calculates 13 of the 14 features found in the paper \"Textural features for image classification\" (Haralick, RM) ";
}

string HaralickFeatureCalculator::GetCurrentParams() {
	stringstream streamer;
	streamer << "It calculates the following features:" << endl;
	for (int feature_ind = 0; feature_ind < (int) feature_names_.size(); ++feature_ind) {
		streamer << " - " << feature_names_[feature_ind] << endl;
	}
	return streamer.str();
}

////////////////////////////////////// Energy Contrast Correlation Feature Calculator Class ///////////////////////////////

EneConCorrFeatureCalculator::EneConCorrFeatureCalculator() {
	feature_names_.push_back("Energy");
	feature_names_.push_back("Contrast");
	feature_names_.push_back("Correlation");
}

EneConCorrFeatureCalculator::~EneConCorrFeatureCalculator() {
}

vector<double> EneConCorrFeatureCalculator::CalculeFeatures(
		const cv::Mat* data) {
	vector<double> ene_con_corr_features(3, 0.0);
	CalculeFeatures(data, &ene_con_corr_features, 0);
	return ene_con_corr_features;
}

void EneConCorrFeatureCalculator::CalculeFeatures(const cv::Mat* data,
		vector<double>* output, int start_index) {
	int cols = data->cols;
	int rows = data->rows;

	const double * glcm_row;
	double mean_x = 0, mean_y = 0, variance_x = 0, variance_y = 0;
	// filling the vectors;
	for (int i = 0; i < rows; i++) {
		glcm_row = data->ptr<double> (i);
		for (int j = 0; j < cols; j++) {
			mean_x += ((i + 1) * glcm_row[j]);
			mean_y += ((j + 1) * glcm_row[j]);
		}
	}

	double contrast = 0, ang2mom = 0, correlation = 0;
	for (int i = 0; i < rows; i++) {
		glcm_row = data->ptr<double> (i);
		for (int j = 0; j < cols; j++) {
			ang2mom += (glcm_row[j] * glcm_row[j]);
			contrast += ((i - j) * (i - j) * glcm_row[j]);
			variance_x += (glcm_row[j] * (i + 1 - mean_x) * (i + 1 - mean_x));
			variance_y += (glcm_row[j] * (j + 1 - mean_y) * (j + 1 - mean_y));
			correlation += ((i + 1) * (j + 1) * glcm_row[j]);
		}
	}
	correlation = (correlation - (mean_x * mean_y)) / sqrt(
			variance_x * variance_y);
	(*output)[start_index++] = sqrt(ang2mom);
	(*output)[start_index++] = contrast;
	(*output)[start_index++] = correlation;
}

int EneConCorrFeatureCalculator::GetNumberOfFeatures() {
	return 3;
}

vector<string> EneConCorrFeatureCalculator::GetFeatureNames() {
	return feature_names_;
}

//Describable Class implementation
string EneConCorrFeatureCalculator::GetClassName() {
	return "EneConCorrFeatureCalculator";
}

string EneConCorrFeatureCalculator::GetComments() {
	return "It calculates 3 features";
}

string EneConCorrFeatureCalculator::GetCurrentParams() {
	stringstream streamer;
	streamer << "It calculates the following features:" << endl;
	for (int feature_ind = 0; feature_ind < (int) feature_names_.size(); ++feature_ind) {
		streamer << " - " << feature_names_[feature_ind] << endl;
	}
	return streamer.str();
}

