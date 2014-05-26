
#include <fstream>
#include <sstream>
#include "texture_analyzer.h"
#include "coo_matrix_generator.h"
#include "feature_calculator.h"

#include "opencv2/opencv.hpp"


///////////////////////////////////////////////// Image Analyzer GLCM /////////////////////////////////////////

ImageAnalyzer::ImageAnalyzer()
{
	// TODO Auto-generated constructor stub

}

ImageAnalyzer::~ImageAnalyzer()
{
	// TODO Auto-generated destructor stub
}




/////////////////////////////////// Abstract Co-occurrence Matrix Texture Analyzer ///////////////////////////////////

AbstractCooMatrixTextureAnalyzer::~AbstractCooMatrixTextureAnalyzer() {
}


AbstractCooMatrixTextureAnalyzer::AbstractCooMatrixTextureAnalyzer(AbstractCooMatrixGeneratorPtr glcm_generator,
		int num_windows_height, int num_windows_width,
		vector<FeatureCalculatorPtr> feature_calculators) {
	coo_matrix_generator_ = glcm_generator;
	num_windows_height_ = num_windows_width;
	num_windows_width_ = num_windows_width;
	feature_calculators_ = feature_calculators;
	windows_separation_height_ = 1;
	windows_separation_width_ = 1;
}

void AbstractCooMatrixTextureAnalyzer::SetNumWindows(int num_windows_height, int num_windows_width){
	num_windows_height_ = num_windows_height;
	num_windows_width_ = num_windows_width;
}


void AbstractCooMatrixTextureAnalyzer::ChangeFeatureCalculators(
		vector<FeatureCalculatorPtr> feature_calculators) {
	feature_calculators_ = feature_calculators;
}

void AbstractCooMatrixTextureAnalyzer::QuantizeGrayMatrix(cv::Mat* image_input_output,
		const int& current_num_levels, const int& desired_num_levels) {

	int height = image_input_output->rows;
	int width = image_input_output->cols;
	int num_channels = image_input_output->channels();
	uchar *data_image;
	for(int i = 0; i < height; i ++){
		data_image = image_input_output->ptr<uchar>(i);
		for (int j = 0; j < width * num_channels; j++){
			double avoiding_overflow = data_image[j];
			data_image[j] = round((avoiding_overflow * (desired_num_levels - 1)) / (255));
		}
	}
	return;
}

void AbstractCooMatrixTextureAnalyzer::SetSeparationWindowsSize(int height, int width) {
	windows_separation_height_ = height;
	windows_separation_width_ = width;
}

void AbstractCooMatrixTextureAnalyzer::GetSeparationWindowsSize(int* height, int* width) {
	*height = windows_separation_height_;
	*width = windows_separation_width_;
}

void AbstractCooMatrixTextureAnalyzer::PrintFeatures(const string &file_to_output,  vector<double> *features, const string &text_before, const string &text_after) {
	//Memory allocation for the feature vector, maybe later I will create another class for managing this
	ofstream xout (file_to_output.c_str(), ios::app);
	if(text_before != "") xout << text_before << " ";
	for(int i = 0 ; i < (int)features->size(); i++){
		xout << (*features)[i] << " ";
	}
	if(text_after != "") xout << " " << text_after;
	xout.close();
}



string AbstractCooMatrixTextureAnalyzer::GetClassName() {
	return "AbstractCooMatrixTextureAnalyzer";
}

string AbstractCooMatrixTextureAnalyzer::GetComments() {
	return "";
}

string AbstractCooMatrixTextureAnalyzer::GetCurrentParams() {
	stringstream streamer;
	streamer << "Num Windows height: " << num_windows_height_  << endl;
	streamer << "Num Windows width: " << num_windows_width_  << endl;
	streamer << "Total Number of features: " << GetNumberOfFeatures() << endl;
	streamer << "Windows separation height: " << windows_separation_height_ << endl;
	streamer << "Windows separation width: " << windows_separation_width_ << endl;
	streamer << coo_matrix_generator_->GetCurrentSetting() << endl;
	for(int feature_calc_ind = 0; feature_calc_ind < (int) feature_calculators_.size(); ++feature_calc_ind){
		streamer << feature_calculators_[feature_calc_ind]->GetCurrentSetting() << endl;
	}
	return streamer.str();
}


////////////////////////////////////////// GrayCooMatrixTextureAnalyzer Class //////////////////////////////////////////////
GrayCooMatrixTextureAnalyzer::GrayCooMatrixTextureAnalyzer(
		AbstractCooMatrixGeneratorPtr coo_matrix_1c_generator, int num_windows_height, int num_windows_width ,
		vector<FeatureCalculatorPtr> feature_calculators):AbstractCooMatrixTextureAnalyzer(coo_matrix_1c_generator, num_windows_height, num_windows_width, feature_calculators) {
}

GrayCooMatrixTextureAnalyzer::~GrayCooMatrixTextureAnalyzer() {
}



void GrayCooMatrixTextureAnalyzer::PerformImageAnalysis_oneImage(
		cv::Mat &image_to_analyze, vector<double>& features) {

}

void GrayCooMatrixTextureAnalyzer::PerformImageAnalysis(
		cv::Mat *image_to_analyze , cv::Mat *feature_vector_to_place, int pos_feature) {
	CV_Assert(image_to_analyze->type() == CV_8UC1) ;

	cv::Mat output_glcm;
	int desired_levels = AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->GetOutLevels();
	AbstractCooMatrixTextureAnalyzer::QuantizeGrayMatrix(image_to_analyze, 256, desired_levels);

	int image_rows = image_to_analyze->rows;
	int image_cols = image_to_analyze->cols;

	int window_height = image_rows + AbstractCooMatrixTextureAnalyzer::windows_separation_height_ * (1 - AbstractCooMatrixTextureAnalyzer::num_windows_height_);
	int window_width =  image_cols + AbstractCooMatrixTextureAnalyzer::windows_separation_width_ * (1 - AbstractCooMatrixTextureAnalyzer::num_windows_width_);
	cout << "rows , columns: " << image_rows << " " << image_cols << " - " << window_height << " " << window_width << endl;

	if(window_height <= 0 || window_width <= 0){
		throw "variables no setted corretly";
	}

	cv::Mat roi_imagen;

	int num_glcms =AbstractCooMatrixTextureAnalyzer::num_windows_height_ * AbstractCooMatrixTextureAnalyzer::num_windows_width_;
	int num_features_per_glcm = 0;
	for(int ind = 0; ind < (int)(feature_calculators_.size()); ind++){
		num_features_per_glcm += feature_calculators_[ind]->GetNumberOfFeatures();
	}
	vector<double> features(num_features_per_glcm * num_glcms, 0.0);

	int start = 0;
	for(int i = 0; i <= image_rows - window_height; i+= AbstractCooMatrixTextureAnalyzer::windows_separation_height_){
		for(int j = 0 ; j <= image_cols - window_width ; j+= AbstractCooMatrixTextureAnalyzer::windows_separation_width_){
			roi_imagen = (*image_to_analyze)(cv::Range(i, i + window_height) , cv::Range(j, j + window_width));
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm, desired_levels, GRAY);

			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
		}
	}
	if(feature_vector_to_place->rows <= pos_feature) throw "Matriz no tiene suficientes filas";
	if(feature_vector_to_place->cols != GetNumberOfFeatures()){
		throw "Matriz no tiene suficientes columnas";
	}
	CV_Assert((int)features.size() == GetNumberOfFeatures());
	float *row_feature_vector = feature_vector_to_place->ptr<float>(pos_feature);
	for(int pos_fea = 0 ; pos_fea < (int)features.size(); ++pos_fea){
		row_feature_vector[pos_fea] = features[pos_fea];
	}
	//PrintFeatures(dest_file, &features, text_before, text_after);

	//return num_features_per_glcm * num_glcms;
}


string GrayCooMatrixTextureAnalyzer::GetClassName() {
	return "GrayCooMatrixTextureAnalyzer";
}

string GrayCooMatrixTextureAnalyzer::GetComments() {
	return "Process gray-level images, classic GLCM calculation";
}

int GrayCooMatrixTextureAnalyzer::GetNumberOfFeatures() {
	int num_features_per_coo_matrix = 0;
	for(int ind = 0; ind < (int)(feature_calculators_.size()); ind++){
		num_features_per_coo_matrix += feature_calculators_[ind]->GetNumberOfFeatures();
	}
	return num_windows_height_ * num_windows_width_ * num_features_per_coo_matrix;
}

string GrayCooMatrixTextureAnalyzer::GetCurrentParams() {
	return AbstractCooMatrixTextureAnalyzer::GetCurrentParams();
}

////////////////////////////////////////// ColorCooMatrixTextureAnalyzer Class ////////////////////////////////////////////
ColorCooMatrixTextureAnalyzer::ColorCooMatrixTextureAnalyzer(
		AbstractCooMatrixGeneratorPtr coo_matrix_1c_generator, int num_windows_height, int num_windows_width ,
		vector<FeatureCalculatorPtr> feature_calculators):AbstractCooMatrixTextureAnalyzer(coo_matrix_1c_generator, num_windows_height, num_windows_width, feature_calculators) {
}

ColorCooMatrixTextureAnalyzer::~ColorCooMatrixTextureAnalyzer() {
}


void ColorCooMatrixTextureAnalyzer::PerformImageAnalysis_oneImage(
		cv::Mat &image_to_analyze, vector<double>& features) {
	CV_Assert(image_to_analyze.type() == CV_8UC3)
		;
	cv::Mat output_glcm;
	int
			desired_levels =
					AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->GetOutLevels();
	AbstractCooMatrixTextureAnalyzer::QuantizeGrayMatrix(&image_to_analyze, 256,
			desired_levels);

	int image_rows = image_to_analyze.rows;
	int image_cols = image_to_analyze.cols;

	int window_height = image_rows
			+ AbstractCooMatrixTextureAnalyzer::windows_separation_height_ * (1
					- AbstractCooMatrixTextureAnalyzer::num_windows_height_);
	int window_width = image_cols
			+ AbstractCooMatrixTextureAnalyzer::windows_separation_width_ * (1
					- AbstractCooMatrixTextureAnalyzer::num_windows_width_);
	if (window_height <= 0 || window_width <= 0) {
		throw "variables no setted corretly";
	}
	//cout << "rows , columns: " << image_rows << " " << image_cols << " - " << window_height << " " << window_width << endl;

	cv::Mat roi_imagen;

	//int number_of_combinations = 9; //RR, RG, RB, GR, ...
	//int num_glcms = number_of_combinations * AbstractCooMatrixTextureAnalyzer::num_windows_height_ * AbstractCooMatrixTextureAnalyzer::num_windows_width_;
	int num_features_per_glcm = 0;
	for (int ind = 0; ind < (int) (feature_calculators_.size()); ind++) {
		num_features_per_glcm
				+= feature_calculators_[ind]->GetNumberOfFeatures();
	}
	//vector<double> features(num_features_per_glcm * num_glcms, 0.0);

	int start = 0, i, j, ind;
	for (i = 0; i <= image_rows - window_height; i
			+= AbstractCooMatrixTextureAnalyzer::windows_separation_height_) {
		for (j = 0; j <= image_cols - window_width; j
				+= AbstractCooMatrixTextureAnalyzer::windows_separation_width_) {
			roi_imagen = image_to_analyze(cv::Range(i, i + window_height),
					cv::Range(j, j + window_width));
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_RR);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_RG);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_RB);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_GR);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_GG);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_GB);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_BR);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_BG);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(
					&roi_imagen, &output_glcm, desired_levels, RGB_BB);
			for (ind = 0; ind < (int) feature_calculators_.size(); ind++) {
				feature_calculators_[ind]->CalculeFeatures(&output_glcm,
						&features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
		}
	}
}

void ColorCooMatrixTextureAnalyzer::PerformImageAnalysis(cv::Mat *image_to_analyze ,
		cv::Mat *feature_vector_to_place, int pos_feature) {
	CV_Assert(image_to_analyze->type() == CV_8UC3);
	cv::Mat output_glcm;
	int desired_levels = AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->GetOutLevels();
	AbstractCooMatrixTextureAnalyzer::QuantizeGrayMatrix(image_to_analyze, 256, desired_levels);


	int image_rows = image_to_analyze->rows;
	int image_cols = image_to_analyze->cols;

	int window_height = image_rows + AbstractCooMatrixTextureAnalyzer::windows_separation_height_ * (1 - AbstractCooMatrixTextureAnalyzer::num_windows_height_);
	int window_width =  image_cols + AbstractCooMatrixTextureAnalyzer::windows_separation_width_ * (1 - AbstractCooMatrixTextureAnalyzer::num_windows_width_);
	if(window_height <= 0 || window_width <= 0){
		throw "variables no setted corretly";
	}
	//cout << "rows , columns: " << image_rows << " " << image_cols << " - " << window_height << " " << window_width << endl;



	cv::Mat roi_imagen;


	int number_of_combinations = 9;
	int num_glcms = number_of_combinations * AbstractCooMatrixTextureAnalyzer::num_windows_height_ *AbstractCooMatrixTextureAnalyzer::num_windows_width_;
	int num_features_per_glcm = 0;
	for(int ind = 0; ind < (int)(feature_calculators_.size()); ind++){
		num_features_per_glcm += feature_calculators_[ind]->GetNumberOfFeatures();
	}
	vector<double> features(num_features_per_glcm * num_glcms, 0.0);

	int start = 0;
	for(int i = 0; i <= image_rows - window_height; i+= AbstractCooMatrixTextureAnalyzer::windows_separation_height_){
		for(int j = 0 ; j <= image_cols - window_width ; j+= AbstractCooMatrixTextureAnalyzer::windows_separation_width_){
			roi_imagen = (*image_to_analyze)(cv::Range(i, i + window_height) , cv::Range(j, j + window_width));
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm, desired_levels, RGB_RR);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_RG);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_RB);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_GR);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_GG);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_GB);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_BR);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_BG);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
			AbstractCooMatrixTextureAnalyzer::coo_matrix_generator_->CalculeCooMatrix(&roi_imagen, &output_glcm,desired_levels, RGB_BB);
			for(int ind = 0 ; ind < (int)feature_calculators_.size(); ind++){
				feature_calculators_[ind]->CalculeFeatures(&output_glcm, &features, start);
				start += feature_calculators_[ind]->GetNumberOfFeatures();
			}
		}
	}
	//pos_feature = NUMBER OF IMAGE
	//GetNumberOfFeatures() = NUMBER OF FEATURES PER IMAGE
	if(feature_vector_to_place->rows <= pos_feature) throw "Matriz (feature_vector_to_place) no tiene suficientes filas (NUMBER OF IMAGES TO ANALIZE)";
	if(feature_vector_to_place->cols != GetNumberOfFeatures()){
		throw "Matriz (feature_vector_to_place) no tiene suficientes columnas (NUMBER OF FEATURES)";
	}

	//cout << features.size() << " " << GetNumberOfFeatures() << endl;
	CV_Assert((int)features.size() == GetNumberOfFeatures());
	float *row_feature_vector = feature_vector_to_place->ptr<float>(pos_feature);
	for(int pos_fea = 0 ; pos_fea < (int)features.size(); ++pos_fea){ // Llenar feature_vector_to_place
		row_feature_vector[pos_fea] = features[pos_fea];
	}

	//OpOthers::show_matrix(*feature_vector_to_place);
}


string ColorCooMatrixTextureAnalyzer::GetClassName() {
	return "ColorCooMatrixTextureAnalyzer";
}

string ColorCooMatrixTextureAnalyzer::GetComments() {
	string comments = "Process Color images especially RGB images, CGLCM calculation from the paper \"Novel method for color textures features extraction based on GLCM\" ";
	comments += "\n There are 9 relations: RR, RG, RB, GR, GG, GB, BR, BG, BB, multiply by 9 the total number of co-occurrence matrices per image parameter";
	return comments;
}

int ColorCooMatrixTextureAnalyzer::GetNumberOfFeatures() {
	int num_features_per_coo_matrix = 0;
	for(int ind = 0; ind < (int)(feature_calculators_.size()); ind++){
		num_features_per_coo_matrix += feature_calculators_[ind]->GetNumberOfFeatures();
	}
	return num_windows_height_ * num_windows_width_ * 9 * num_features_per_coo_matrix;
}

string ColorCooMatrixTextureAnalyzer::GetCurrentParams() {
	return  AbstractCooMatrixTextureAnalyzer::GetCurrentParams();
}

