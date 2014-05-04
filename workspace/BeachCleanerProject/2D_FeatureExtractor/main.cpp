#include <iostream>
#include "texture_analyzer.h"
#include "feature_calculator.h"
#include "coo_matrix_generator.h"
#include "texture_analyzer_factories.h"
#include "file_format_manager.h"
#include "batch_processor.h"

using namespace std;

void getFeaturesDB() {

	const string database_dir_in = "/home/rgap/BD/BD_IMGS_pred";
	const string database_dir_out = "/home/rgap/BD/BD_IMGS_feat_pred";
	const string groupName = "GControl";

	PredefinedFormat extractor(database_dir_in, database_dir_out);
	FeaturesFormatManagerPtr formato(new PredefinedFormatManager());
	FeatureMerger *merger = new FeatureMerger(database_dir_out,
			database_dir_out);
	merger->AddAllowedFileExtension(".vec");

	string gtemp;
	stringstream ss;
	for (int ind = 1; ind <= 4; ++ind) { //Num Descriptor
		ColorCooMatrixTextureAnalyzerPtr image_analyzer =
				TextureAnalyzerFactories::ColorEneConCorrTextureAnalyzer(ind,
						64, 1, 1);
		extractor.SetImageAnalyzer(image_analyzer);
		extractor.SetClassificationFormatManager(formato);

		ss << ind;
		gtemp = ss.str();
		extractor.PerformBatching(gtemp);
		ss.str(std::string());
	}

	// FEATURES MERGED FILE
	merger->SetClassificationFormatManager(formato);
	merger->PerformBatching(groupName);
	delete merger;

}

void getFeaturesOneImage() {

	const string
			image_path =
					"/home/rgap/REPOSITORIES/rgap-Computer-Vision-Codes/CONTEST/PATTERN_RECOGNITION/DB/can2.png";

	OneImageFeatureExtractor extractor;

	////////////
	ColorCooMatrixTextureAnalyzerPtr image_analyzer =
			TextureAnalyzerFactories::ColorEneConCorrTextureAnalyzer(2, 64, 1,
					1);

	extractor.ReadImage(image_path);
	extractor.SetImageAnalyzer(image_analyzer);

	int num_features = image_analyzer->GetNumberOfFeatures();

	vector<double> features(num_features);
	extractor.PerformProcessOneImage(features);

	cout<<num_features<<endl;
	for (int i = 0; i < num_features; ++i) {
		cout<<features[i]<<" ";
	}

}

/*
int main(int argc, char** argv) {
	getFeaturesOneImage();
	return 1;
}
*/
