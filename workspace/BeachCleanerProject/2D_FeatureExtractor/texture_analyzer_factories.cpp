
#include "texture_analyzer_factories.h"
#include "texture_analyzer.h"
#include "feature_calculator.h"
#include "coo_matrix_generator.h"

TextureAnalyzerFactories::TextureAnalyzerFactories() {
	// TODO Auto-generated constructor stub

}

TextureAnalyzerFactories::~TextureAnalyzerFactories() {
	// TODO Auto-generated destructor stub
}

ColorCooMatrixTextureAnalyzerPtr TextureAnalyzerFactories::ColorEneConCorrTextureAnalyzer(
		int distance_relation, int max_pixels_value, int num_windows_height,
		int num_windows_width) {

	vector<FeatureCalculatorPtr> features;
	features.push_back(FeatureCalculatorPtr(new EneConCorrFeatureCalculator()));

	vector < pair<int, int> > offset;
	offset.push_back(make_pair(0, distance_relation));
	offset.push_back(make_pair(distance_relation, 0));
	offset.push_back(make_pair(distance_relation, distance_relation));
	offset.push_back(make_pair(-distance_relation, distance_relation));

	bool symmetric_flag = true;
	int desired_gray_levels = max_pixels_value;
	AbstractCooMatrixGeneratorPtr glcm_generator(
			new CooMatrixGeneratorFrom3CImage(offset, symmetric_flag,
					desired_gray_levels));

	vector<int> banned;
	banned.push_back(desired_gray_levels - 1);

	glcm_generator->SetBannedValues(banned);

	ColorCooMatrixTextureAnalyzerPtr analyzer(
			new ColorCooMatrixTextureAnalyzer(glcm_generator,
					num_windows_height, num_windows_width, features));
	return analyzer;
}

GrayCooMatrixTextureAnalyzerPtr TextureAnalyzerFactories::GrayEneConCorrTextureAnalyzer(
		int distance_relation, int max_pixels_value, int num_windows_height,
		int num_windows_width) {
	vector<FeatureCalculatorPtr> features;
	features.push_back(FeatureCalculatorPtr(new EneConCorrFeatureCalculator()));

	vector < pair<int, int> > offset;
	offset.push_back(make_pair(0, distance_relation));
	offset.push_back(make_pair(distance_relation, 0));
	offset.push_back(make_pair(distance_relation, distance_relation));
	offset.push_back(make_pair(-distance_relation, distance_relation));

	bool symmetric_flag = true;
	int desired_gray_levels = max_pixels_value + 1;
	AbstractCooMatrixGeneratorPtr glcm_generator(
			new CooMatrixGeneratorFrom1CImage(offset, symmetric_flag,
					desired_gray_levels));

	vector<int> banned;
	banned.push_back(desired_gray_levels - 1);

	glcm_generator->SetBannedValues(banned);

	GrayCooMatrixTextureAnalyzerPtr analyzer(
			new GrayCooMatrixTextureAnalyzer(glcm_generator,
					num_windows_height, num_windows_width, features));
	return analyzer;
}

