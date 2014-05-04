#include "coo_matrix_generator.h"

#include <sstream>

#include <opencv2/opencv.hpp>

AbstractCooMatrixGenerator::AbstractCooMatrixGenerator() {
	offset_.push_back(make_pair(1, 0));
	symmetric_ = true;
	out_levels_ = 16;
	banned_values_ = new bool[NUM_OF_LEVELS];
	fill(banned_values_, banned_values_ + NUM_OF_LEVELS, 0);
}

AbstractCooMatrixGenerator::AbstractCooMatrixGenerator(
		vector<pair<int, int> > offset, bool symmetric,
		int out_levels) {
	offset_ = offset;
	symmetric_ = symmetric;
	out_levels_ = out_levels;
	banned_values_ = new bool[NUM_OF_LEVELS];
	fill(banned_values_, banned_values_ + NUM_OF_LEVELS, 0);
}

AbstractCooMatrixGenerator::~AbstractCooMatrixGenerator() {
	delete[] banned_values_;
}

void AbstractCooMatrixGenerator::SetOffset(
		vector<pair<int, int> > offset) {
	offset_ = offset;
}

void AbstractCooMatrixGenerator::SetSymmetricFlag(bool symmetric) {
	symmetric_ = symmetric;
}

vector<pair<int, int> > AbstractCooMatrixGenerator::GetOffset() {
	return offset_;
}

void AbstractCooMatrixGenerator::SetBannedValues(
		vector<int> banned_gray_values) {

	fill(banned_values_, banned_values_ + NUM_OF_LEVELS, false);
	for (int i = 0; i < (int) banned_gray_values.size(); i++) {
		banned_values_[banned_gray_values[i] % out_levels_] = true;
	}
}

vector<int> AbstractCooMatrixGenerator::GetBannedValues() {
	vector<int> output;
	for (int i = 0; i < NUM_OF_LEVELS; i++) {
		if (banned_values_[i])
			output.push_back(i);
	}
	return output;
}

void AbstractCooMatrixGenerator::SetOutLevels(int out_levels) {
	out_levels_ = out_levels;
}

int AbstractCooMatrixGenerator::GetOutLevels() {
	return out_levels_;
}

string AbstractCooMatrixGenerator::GetClassName() {
	return "AbstractCooMatrixGenerator";
}

string AbstractCooMatrixGenerator::GetComments() {
	return "";
}

string AbstractCooMatrixGenerator::GetCurrentParams() {
	stringstream streamer;
	streamer << "List of Offsets: ";
	for (int ind = 0; ind < (int) offset_.size(); ind++) {
		if (ind != 0)
			streamer << ", ";
		streamer << "[" << offset_[ind].first << ", " << offset_[ind].second
				<< "]";
	}
	streamer << endl;
	streamer << "Dimension of the Co-occurrence Matrix: " << out_levels_
			<< endl;
	streamer << "Symmetric Flag: " << symmetric_ << endl;
	streamer << "Banned Pixel Values: ";
	bool flag_used = false;
	for (int ind = 0; ind < NUM_OF_LEVELS; ++ind) {
		if (banned_values_[ind]) {
			if (flag_used == true)
				streamer << ", ";
			streamer << ind;
			flag_used = true;
		}
	}
	streamer << endl;
	return streamer.str();
}

////////////////////////////////////////// CooMatrixGeneratorFrom3CImage class /////////////////////////////////////////

CooMatrixGeneratorFrom3CImage::CooMatrixGeneratorFrom3CImage() :
	AbstractCooMatrixGenerator() {
}

CooMatrixGeneratorFrom3CImage::~CooMatrixGeneratorFrom3CImage() {
}

CooMatrixGeneratorFrom3CImage::CooMatrixGeneratorFrom3CImage(
		vector<pair<int, int> > offset, bool symmetric,
		int out_levels) :
	AbstractCooMatrixGenerator(offset, symmetric, out_levels) {

}

void CooMatrixGeneratorFrom3CImage::QuantizeMatrix(cv::Mat* image_input_output,
		const int& current_num_levels, const int& desired_num_levels) {
	int height = image_input_output->rows;
	int width = image_input_output->cols;
	int num_channels = image_input_output->channels();
	uchar *data_image;
	for (int i = 0; i < height; i++) {
		data_image = image_input_output->ptr<uchar> (i);
		for (int j = 0; j < width * num_channels; j++) {
			double avoiding_overflow = data_image[j];
			data_image[j] = round(
					(avoiding_overflow * (desired_num_levels - 1)) / (255));
		}
	}
	return;
}

void CooMatrixGeneratorFrom3CImage::CalculeCooMatrix(cv::Mat* image_input,
		cv::Mat* output_glcm, int in_num_levels, GLCM_RELATION relation) {

	CV_Assert(image_input->type() == CV_8UC3)
		;
	int channel_ref = -1, channel_nei = -1;
	int relation_num = relation;
	int temp = relation_num >> 3;
	channel_nei = temp >> 1;
	relation_num -= (temp << 3);
	channel_ref = relation_num >> 1;

	int sum_all = 0;
	int height = image_input->rows;
	int width = image_input->cols;
	int size_offset = offset_.size();

	if (in_num_levels != out_levels_)
		QuantizeMatrix(image_input, in_num_levels, out_levels_);
	*output_glcm = cv::Mat::zeros(
			cv::Size(AbstractCooMatrixGenerator::out_levels_,
					AbstractCooMatrixGenerator::out_levels_), CV_64FC1);

	for (int i = 0; i < height; i++) {
		uchar * row_image_input = image_input->ptr<uchar> (i);
		for (int j = 0; j < width; j++) {

			uchar pix_ref_val = row_image_input[channel_ref + j * 3];
			if (AbstractCooMatrixGenerator::banned_values_[pix_ref_val])
				continue;

			for (int k = 0; k < size_offset; k++) {
				int neighbor_i = i + offset_[k].first;
				int neighbor_j = j + offset_[k].second;
				if (!(neighbor_i >= 0 && neighbor_i < height && neighbor_j >= 0
						&& neighbor_j < width))
					continue;

				uchar pix_nei_val = image_input->at<cv::Vec3b> (neighbor_i,
						neighbor_j)[channel_nei];

				if (AbstractCooMatrixGenerator::banned_values_[pix_nei_val])
					continue;

				output_glcm->at<double> (pix_ref_val, pix_nei_val)
						= output_glcm->at<double> (pix_ref_val, pix_nei_val)
								+ 1.0;
				if (symmetric_)
					output_glcm->at<double> (pix_nei_val, pix_ref_val)
							= output_glcm->at<double> (pix_nei_val, pix_ref_val)
									+ 1.0;
				sum_all += 1 + symmetric_;
			}
		}
	}

	cv::divide(*output_glcm, sum_all, *output_glcm);

	return;

}

string CooMatrixGeneratorFrom3CImage::GetClassName() {
	return "CooMatrixGeneratorFrom3CImage";
}

string CooMatrixGeneratorFrom3CImage::GetComments() {
	return "It generates co-occurrence matrices from three-channel images";
}

string CooMatrixGeneratorFrom3CImage::GetCurrentParams() {
	return AbstractCooMatrixGenerator::GetCurrentParams();
}

//////////////////////////////////////////// CooMatrixGeneratorFrom1CImage class ///////////////////////////////////////////


CooMatrixGeneratorFrom1CImage::CooMatrixGeneratorFrom1CImage() :
	AbstractCooMatrixGenerator() {
}

CooMatrixGeneratorFrom1CImage::CooMatrixGeneratorFrom1CImage(
		vector<pair<int, int> > offset, bool symmetric,
		int out_levels) :
	AbstractCooMatrixGenerator(offset, symmetric, out_levels) {
}

CooMatrixGeneratorFrom1CImage::~CooMatrixGeneratorFrom1CImage() {
}

void CooMatrixGeneratorFrom1CImage::CalculeCooMatrix(cv::Mat* image_input,
		cv::Mat* output_glcm, int in_num_levels, GLCM_RELATION relation) {
	CV_Assert(image_input->type() == CV_8UC1)
		;

	int sum_all = 0;

	if (in_num_levels != AbstractCooMatrixGenerator::out_levels_)
		QuantizeMatrix(image_input, in_num_levels,
				AbstractCooMatrixGenerator::out_levels_);

	int height = image_input->rows;
	int width = image_input->cols;
	int size_offset = offset_.size();
	*output_glcm = cv::Mat::zeros(
			cv::Size(AbstractCooMatrixGenerator::out_levels_,
					AbstractCooMatrixGenerator::out_levels_), CV_64FC1);

	for (int i = 0; i < height; i++) {
		uchar * row_image_input = image_input->ptr<uchar> (i);
		for (int j = 0; j < width; j++) {
			uchar pix_ref_val = row_image_input[j];
			if (AbstractCooMatrixGenerator::banned_values_[pix_ref_val])
				continue;

			for (int k = 0; k < size_offset; k++) {
				int neighbor_i = i + offset_[k].first;
				int neighbor_j = j + offset_[k].second;
				if (!(neighbor_i >= 0 && neighbor_i < height && neighbor_j >= 0
						&& neighbor_j < width))
					continue;

				uchar pix_nei_val = image_input->at<uchar> (neighbor_i,
						neighbor_j);

				if (AbstractCooMatrixGenerator::banned_values_[pix_nei_val])
					continue;

				output_glcm->at<double> (pix_ref_val, pix_nei_val)
						= output_glcm->at<double> (pix_ref_val, pix_nei_val)
								+ 1.0;
				if (symmetric_)
					output_glcm->at<double> (pix_nei_val, pix_ref_val)
							= output_glcm->at<double> (pix_nei_val, pix_ref_val)
									+ 1.0;

				sum_all += 1 + symmetric_;
			}
		}
	}

	cv::divide(*output_glcm, sum_all, *output_glcm);
	return;
}

void CooMatrixGeneratorFrom1CImage::QuantizeMatrix(cv::Mat* image_input_output,
		const int& current_num_levels, const int& desired_num_levels) {

	int height = image_input_output->rows;
	int width = image_input_output->cols;
	int num_channels = image_input_output->channels();
	uchar *data_image;
	for (int i = 0; i < height; i++) {
		data_image = image_input_output->ptr<uchar> (i);
		for (int j = 0; j < width * num_channels; j++) {
			double avoiding_overflow = data_image[j];
			data_image[j] = round(
					(avoiding_overflow * (desired_num_levels - 1)) / (255));
		}
	}
	return;
}

string CooMatrixGeneratorFrom1CImage::GetClassName() {
	return "CooMatrixGeneratorFrom1CImage";
}

string CooMatrixGeneratorFrom1CImage::GetComments() {
	return "It generates co-occurrence matrices from one-channel images";
}

string CooMatrixGeneratorFrom1CImage::GetCurrentParams() {
	return AbstractCooMatrixGenerator::GetCurrentParams();
}

