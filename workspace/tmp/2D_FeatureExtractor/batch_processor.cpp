#include <iomanip>

#include <cstdlib>
#include <iostream>
#include <queue>
#include <sstream>
#include <fstream>
#include <set>
#include <ctime>

#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <opencv2/opencv.hpp>

#include "batch_processor.h"
#include "texture_analyzer.h"
#include "file_format_manager.h"

using namespace std;

////////////////////////////////////////// AbstractOneImageProcessor Class ////////////////////////////////////

AbstractOneImageProcessor::AbstractOneImageProcessor() {
}

AbstractOneImageProcessor::~AbstractOneImageProcessor() {
}

void AbstractOneImageProcessor::ReadImage(string path_image) throw (exception) {
	path_image_ = path_image;
	image_ = cv::imread(path_image_);
}

void AbstractOneImageProcessor::SetImage(cv::Mat &image){
	image_ = image;
}

cv::Mat &AbstractOneImageProcessor::GetImage(){
	return image_;
}

void AbstractOneImageProcessor::PerformProcessOneImage(vector<double>& features) throw (exception) {
	ProcessOneImage(features);
}

////////////////////////////////////////// OneImageFeatureExtractor Class ////////////////////////////////////


OneImageFeatureExtractor::OneImageFeatureExtractor(){
}

void OneImageFeatureExtractor::ReadImage(string path_image) throw (exception) {
	AbstractOneImageProcessor::ReadImage(path_image);
}

void OneImageFeatureExtractor::SetImage(cv::Mat &image){
	AbstractOneImageProcessor::SetImage(image);
}

OneImageFeatureExtractor::~OneImageFeatureExtractor() {
}

void OneImageFeatureExtractor::SetImageAnalyzer(ImageAnalyzerPtr image_analyzer) {
	image_analyzer_ = image_analyzer;
}

void OneImageFeatureExtractor::ProcessOneImage(vector<double>& features) throw (exception) {

	image_analyzer_->PerformImageAnalysis_oneImage(AbstractOneImageProcessor::GetImage() , features);
}

////////////////////////////////////////// AbstractBatchProcessor Class ////////////////////////////////////

AbstractBatchProcessor::AbstractBatchProcessor() {
	prefix_dir_ = "DEFAULT_PREFIX";
}

AbstractBatchProcessor::~AbstractBatchProcessor() {
}

void AbstractBatchProcessor::SetDbDirToProcess(string path_db_directory)
		throw (exception) {
	if (CheckIfDirectoryExists(path_db_directory) == false)
		throw ExDirNoFound();
	path_database_directory_ = path_db_directory;
}

string AbstractBatchProcessor::GetDbDirToProcess() {
	return path_database_directory_;
}

void AbstractBatchProcessor::SetDirToPlaceResults(
		string path_placement_directory) throw (exception) {
	if (CheckIfDirectoryExists(path_placement_directory) == false)
		throw ExDirNoFound();
	path_placement_directory_ = path_placement_directory;
}

string AbstractBatchProcessor::GetDirToPlaceResults() {
	return path_placement_directory_;
}

void AbstractBatchProcessor::AddAllowedFileExtension(
		std::string allowed_file_extension) throw (std::exception) {
	if ((int) allowed_file_extension.size() > 2) {
		if (!(allowed_file_extension[0] == '.' && allowed_file_extension.find(
				".", 1) == std::string::npos))
			throw ExBadFormat();
	} else
		return throw ExBadFormat();
	allowed_file_extensions_.push_back(allowed_file_extension);
}

void AbstractBatchProcessor::PerformBatching(const string & groupName)
		throw (exception) {

	string unique_name;

	if (groupName.empty()) {
		unique_name = prefix_dir_ + "_" + GenerateUniqueDirName();
	} else {
		unique_name = prefix_dir_ + "_" + groupName;
	}

	string dir_to_place = path_placement_directory_ + "/" + unique_name;
	cout << dir_to_place << endl;
	cout << unique_name << endl;

	string cmd_remove_dirs = "rm " + dir_to_place + "/*";
	system(cmd_remove_dirs.c_str());
	mkdir(dir_to_place.c_str(), 0775);

	if ((int) allowed_files_.size() == 0)
		RetrieveAllAllowedFiles();

	sort(allowed_files_.begin(), allowed_files_.end());
	ProcessDatabaseFiles(allowed_files_, dir_to_place);
	sleep(1);
}

void AbstractBatchProcessor::ExploreDir(const string& dir_to_explore,
		vector<pair<string, unsigned char> >* list_of_results) {
	list_of_results->clear();
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir_to_explore.c_str())) == NULL) { //todos en directorio actual
		cout << "Error(" << errno << ") opening " << dir_to_explore << endl;
		return;
	}

	while ((dirp = readdir(dp)) != NULL) {
		list_of_results->push_back(
				make_pair(string(dirp->d_name), dirp->d_type));
	}
	closedir(dp);
}

bool AbstractBatchProcessor::ValidateDirectory(const string& directory_name) {
	if ((int) directory_name.size() <= 0 || directory_name[0] == '.')
		return false;
	return true;
}

bool AbstractBatchProcessor::ValidateArchive(const std::string& archive_name) {
	bool match_ext = false;
	if ((int) allowed_file_extensions_.size() == 0)
		match_ext = true;
	for (int pos = 0; pos < (int) allowed_file_extensions_.size(); ++pos) {
		size_t pos_ext = archive_name.rfind(allowed_file_extensions_[pos]);
		if (pos_ext != std::string::npos) {
			if ((pos_ext + allowed_file_extensions_[pos].size())
					== archive_name.size()) {
				match_ext = true;
				break;
			}
		}
	}
	if (match_ext == false)
		return false;

	bool must_cent = false, banned_cent = false;
	if ((int) must_patterns_.size() == 0)
		must_cent = true;

	for (int ind_must = 0; ind_must < (int) must_patterns_.size(); ++ind_must) {
		std::size_t must_match = archive_name.find(must_patterns_[ind_must]);
		if (must_match != std::string::npos)
			must_cent = true;
	}
	for (int ind_banned = 0; ind_banned < (int) banned_patterns_.size(); ++ind_banned) {
		std::size_t banned_match = archive_name.find(
				banned_patterns_[ind_banned]);
		if (banned_match != std::string::npos)
			banned_cent = true;
	}
	if (banned_cent == true)
		return false;
	return must_cent;
}

void AbstractBatchProcessor::RetrieveAllAllowedFiles() {
	allowed_files_.clear();
	std::queue < std::pair<std::string, int> > cola;
	std::vector < std::pair<std::string, unsigned char> > temporal;
	cola.push(std::make_pair(path_database_directory_, 0));
	int max_depth_ = 2;
	while (!cola.empty()) {
		int depth = cola.front().second;
		std::string current_directory = cola.front().first;
		ExploreDir(current_directory, &temporal);
		cola.pop();
		//cout << "temporal.size() = " << temporal.size() << endl;
		for (int ind_name = 0; ind_name < (int) temporal.size(); ++ind_name) {
			//cout << "temporal[ind_name].first - " << temporal[ind_name].first << endl;
			if (temporal[ind_name].second == DT_DIR && ValidateDirectory( // ES DIRECTORIO
					temporal[ind_name].first) && (depth + 1 <= max_depth_)) {
				cola.push(
						std::make_pair(
								current_directory + "/"
										+ temporal[ind_name].first, depth + 1));

				//cout << "PUSH" << endl;
				//cout << current_directory + "/" + temporal[ind_name].first << endl;

			} else if (temporal[ind_name].second == DT_REG && ValidateArchive( // ES ARCHIVO
					temporal[ind_name].first)) {
				allowed_files_.push_back(
						current_directory + "/" + temporal[ind_name].first);
			}
		}
	}
}

bool AbstractBatchProcessor::CheckIfDirectoryExists(const string& dir_to_check) {
	DIR *dp;
	if ((dp = opendir(dir_to_check.c_str())) == NULL)
		return false;
	return true;
}

void AbstractBatchProcessor::SetPrefixDir(const string& prefix_dir) {
	prefix_dir_ = prefix_dir;
}

string AbstractBatchProcessor::GenerateUniqueDirName() {
	time_t rawtime;
	struct tm * aTime;
	string months_names[] = { "Ene", "Feb", "Mar", "Abr", "May", "Jun", "Jul",
			"Ago", "Set", "Oct", "Nov", "Dic" };
	time(&rawtime);
	aTime = localtime(&rawtime);

	int day = aTime->tm_mday;
	int month = aTime->tm_mon; // Month is 0 - 11, add 1 to get a jan-dec 1-12 concept
	int year = aTime->tm_year + 1900; // Year is # years since 1900
	int hour = aTime->tm_hour;
	int minute = aTime->tm_min;
	int second = aTime->tm_sec;
	int process_id = getpid();
	stringstream streamer_name_dir;
	streamer_name_dir << "BAT_" << setfill('0') << setw(2) << day << "-";
	streamer_name_dir << setfill('0') << setw(2) << months_names[month] << "-"
			<< year << "--";
	streamer_name_dir << setfill('0') << setw(2) << hour << "h-";
	streamer_name_dir << setfill('0') << setw(2) << minute << "m-";
	streamer_name_dir << setfill('0') << setw(2) << second << "s--P"
			<< process_id;
	return streamer_name_dir.str();
}

////////////////////////////////////////// Image Batch Vectorizer Class ////////////////////////////////////

ImageFeatureExtractor::ImageFeatureExtractor(const string& main_dir_to_process,
		const string& main_dir_place_results) throw (exception) :
	AbstractBatchProcessor() {

	SetPrefixDir("IMG_FEATURES");
	AbstractBatchProcessor::SetDbDirToProcess(main_dir_to_process);
	AbstractBatchProcessor::SetDirToPlaceResults(main_dir_place_results);

	features_format_manager_ = FeaturesFormatManagerPtr(
			new PredefinedFormatManager());
}

ImageFeatureExtractor::~ImageFeatureExtractor() {
}

void ImageFeatureExtractor::ProcessDatabaseFiles( // IMAGES FEATURES
		const vector<string>& allowed_files, const string& dir_to_place)
		throw (exception) {
	string training_file = "Training_feature_vector.vec";
	string desc_ext = ".desc";
	string category = "", unique_name = "";
	cv::Mat image;
	int number_of_images = allowed_files.size();
	int number_of_features = image_analyzer_->GetNumberOfFeatures();

	cv::Mat feature_vectors = cv::Mat(number_of_images, number_of_features,
			CV_32FC1); //ROWS = IMAGES, COLUMNS = FEATURES
	vector < string > list_of_names;
	vector < string > list_of_categories;
	list_of_names.resize(number_of_images, "");
	list_of_categories.resize(number_of_images, "");
	for (int ind = 0; ind < number_of_images; ++ind) {
		ReadCategoryAndUniqueName(allowed_files[ind], &category, &unique_name);
		list_of_names[ind] = unique_name;
		list_of_categories[ind] = category;
		cout << allowed_files[ind] << " " << category << " " << unique_name
				<< endl;
		image = cv::imread(allowed_files[ind].c_str()); // Actual Image
		try {
			image_analyzer_->PerformImageAnalysis(&image, &feature_vectors, ind);
		} catch (const char* msg) {
			cout << msg << endl;
		}
	}
	ofstream desc_out((dir_to_place + "/" + training_file + desc_ext).c_str());
	features_format_manager_->Print(dir_to_place + "/" + training_file,
			list_of_names, list_of_categories, feature_vectors);
	desc_out << image_analyzer_->GetCurrentSetting() << endl;
	desc_out.close();
}

void ImageFeatureExtractor::SetImageAnalyzer(ImageAnalyzerPtr image_analyzer) {
	image_analyzer_ = image_analyzer;
}

void ImageFeatureExtractor::SetClassificationFormatManager(
		FeaturesFormatManagerPtr features_format_manager) {
	features_format_manager_ = features_format_manager;
}

//////////////////////////// Image Batch Vectorizer for Cecovasa Format Files /////////////////////////////////


PredefinedFormat::PredefinedFormat(const string& main_dir_to_process,
		const string& main_dir_place_results) throw (exception) :
	ImageFeatureExtractor(main_dir_to_process, main_dir_place_results) {
}

PredefinedFormat::~PredefinedFormat() {
}

void PredefinedFormat::ReadCategoryAndUniqueName(
		const string& image_name_to_parse, string* category,
		string* unique_name) {
	size_t pos_slash = image_name_to_parse.rfind("/");
	pos_slash++;
	int ind = pos_slash;
	while (!(image_name_to_parse[ind] >= '0' && image_name_to_parse[ind] <= '9'))
		ind++;
	(*unique_name) = image_name_to_parse.substr(pos_slash);
	(*category) = image_name_to_parse.substr(pos_slash, ind - pos_slash);
}

///////////////////////////////////// Batch Feature Merger Class /////////////////////////////////////////
FeatureMerger::FeatureMerger(const string& main_dir_to_process,
		const string& main_dir_place_results) throw (exception) :
	AbstractBatchProcessor() {
	SetPrefixDir("FEATURE_MERGER");
	features_format_manager_ = FeaturesFormatManagerPtr(
			new PredefinedFormatManager());
	AbstractBatchProcessor::SetDbDirToProcess(main_dir_to_process);
	AbstractBatchProcessor::SetDirToPlaceResults(main_dir_place_results);
}

FeatureMerger::~FeatureMerger() {
}

void FeatureMerger::SetClassificationFormatManager(
		FeaturesFormatManagerPtr features_format_manager) {
	features_format_manager_ = features_format_manager;
}

void FeatureMerger::ProcessDatabaseFiles(const vector<string>& allowed_files,
		const string& dir_to_place) throw (exception) {
	cout << "# Allowed files = " << allowed_files.size() << endl;
	string prefix = "FUSION_";
	string training_file = "Training_feature_vector.vec";
	string desc_ext = ".desc";
	vector < string > input_names;
	vector < string > input_classes;
	vector<boost::shared_ptr<cv::Mat> > features_per_input;
	vector < string > distinct_classes;
	int number_of_features = 0;
	int number_of_inputs = 0;
	ofstream desc_stream_out;
	for (int pos = 0; pos < (int) allowed_files.size(); ++pos) {
		cout << allowed_files[pos] << endl;
		ifstream desc_stream_in((allowed_files[pos] + desc_ext).c_str());
		if (!desc_stream_in.fail()) {
			desc_stream_out.open(
					(dir_to_place + "/" + prefix + training_file + desc_ext).c_str(),
					ios::app);
			string temp_line = "";
			while (!desc_stream_in.eof()) {
				getline(desc_stream_in, temp_line);
				desc_stream_out << temp_line << endl;
			}
			desc_stream_out << endl;
			desc_stream_out.close();
			desc_stream_in.close();
		}
		boost::shared_ptr<cv::Mat> features(new cv::Mat(1, 1, CV_32FC1));
		features_format_manager_->Retrieve(allowed_files[pos], &input_names,
				&input_classes, features.get(), &distinct_classes);
		number_of_features += features_format_manager_->GetNumberOfFeatures();
		number_of_inputs = features_format_manager_->GetNumberOfInputs();
		features_per_input.push_back(features);
	}
	cv::Mat output_features = cv::Mat(number_of_inputs, number_of_features,
			CV_32FC1);
	int start_pos_feature = 0;
	for (int pos = 0; pos < (int) features_per_input.size(); ++pos) {
		if (features_per_input[pos]->rows != number_of_inputs)
			throw ExMismatchParams();
		for (int ind = 0; ind < number_of_inputs; ++ind) {
			float *row_input_feature =
					features_per_input[pos]->ptr<float> (ind);
			float *row_output_feature = output_features.ptr<float> (ind);
			for (int fea_pos = 0; fea_pos < features_per_input[pos]->cols; ++fea_pos) {
				row_output_feature[start_pos_feature + fea_pos]
						= row_input_feature[fea_pos];
			}
		}
		start_pos_feature += features_per_input[pos]->cols;
	}
	features_format_manager_->Print(
			dir_to_place + "/" + prefix + training_file, input_names,
			input_classes, output_features);

}
