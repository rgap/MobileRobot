#ifndef BATCH_PROCESSOR_H_
#define BATCH_PROCESSOR_H_

#include "typedefs.h"
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "exceptions.h"

using namespace std;

class AbstractOneImageProcessor {

public:
	AbstractOneImageProcessor();
	virtual ~AbstractOneImageProcessor();
	virtual void PerformProcessOneImage(vector<double>& features) throw (exception);
	virtual void ReadImage(string path_image)
				throw (exception);
	cv::Mat& GetImage();
	void SetImage(cv::Mat &image);
protected:
	virtual void ProcessOneImage(vector<double>& features) throw (exception) = 0;
private:
	string path_image_;
	cv::Mat image_;
	vector<double> features_;
};

class OneImageFeatureExtractor: public AbstractOneImageProcessor {
public:
	OneImageFeatureExtractor();
	virtual ~OneImageFeatureExtractor();
	void ReadImage(string path_image) throw (exception) ;

	void SetImage(cv::Mat &image);

	void SetImageAnalyzer(ImageAnalyzerPtr image_analyzer);
protected:
	virtual void ProcessOneImage(vector<double>& features) throw (exception);
private:
	ImageAnalyzerPtr image_analyzer_;
};

class AbstractBatchProcessor {
public:
	AbstractBatchProcessor();
	virtual ~AbstractBatchProcessor();
	virtual void SetDbDirToProcess(string path_db_directory) throw (exception);
	virtual string GetDbDirToProcess();
	virtual void SetDirToPlaceResults(string path_placement_directory)
			throw (exception);
	virtual string GetDirToPlaceResults();
	virtual void PerformBatching(const string & groupName) throw (exception);

	virtual void AddAllowedFileExtension(string allowed_file_extension)
			throw (exception);
	string GenerateUniqueDirName();

protected:
	virtual void ProcessDatabaseFiles(const vector<string> & allowed_files,
			const string &dir_to_place) throw (exception) = 0;

	virtual void SetPrefixDir(const string &prefix_dir);

private:
	string path_database_directory_;
	string path_placement_directory_;
	string prefix_dir_;

	vector<string> allowed_files_;
	vector<string> allowed_file_extensions_;

	vector<string> must_patterns_;
	vector<string> banned_patterns_;

	bool CheckIfDirectoryExists(const string & dir_to_check);
	void ExploreDir(const string &dir_to_explore,
			vector<pair<string, unsigned char> > *list_of_results);
	bool ValidateArchive(const string &archive_name);
	bool ValidateDirectory(const string &directory_name);
	void RetrieveAllAllowedFiles();
};

class ImageFeatureExtractor: public AbstractBatchProcessor {
public:
	ImageFeatureExtractor(const string &main_dir_to_process,
			const string &main_dir_place_results) throw (exception);
	virtual ~ImageFeatureExtractor();
	virtual void ReadCategoryAndUniqueName(const string &image_name_to_parse,
			string *category, string *unique_name) = 0;
	void SetImageAnalyzer(ImageAnalyzerPtr image_analyzer);
	void SetClassificationFormatManager(
			FeaturesFormatManagerPtr features_format_manager);
protected:
	virtual void ProcessDatabaseFiles(const vector<string> & allowed_files,
			const string &dir_to_place) throw (exception);
private:
	ImageAnalyzerPtr image_analyzer_;
	FeaturesFormatManagerPtr features_format_manager_;
};

class PredefinedFormat: public ImageFeatureExtractor {
public:
	using ImageFeatureExtractor::SetImageAnalyzer;
	using ImageFeatureExtractor::SetClassificationFormatManager;
	using ImageFeatureExtractor::ProcessDatabaseFiles;
	PredefinedFormat(const string &main_dir_to_process,
			const string &main_dir_place_results) throw (exception);
	virtual ~PredefinedFormat();
	virtual void ReadCategoryAndUniqueName(const string &image_name_to_parse,
			string *category, string *unique_name);
};

class FeatureMerger: public AbstractBatchProcessor {
public:
	FeatureMerger(const string& main_dir_to_process,
			const string& main_dir_place_results) throw (exception);
	virtual ~FeatureMerger();
	void SetClassificationFormatManager(
			FeaturesFormatManagerPtr features_format_manager);
protected:
	virtual void ProcessDatabaseFiles(const vector<string> & allowed_files,
			const string &dir_to_place) throw (exception);
private:
	FeaturesFormatManagerPtr features_format_manager_;
};

#endif /* BATCH_PROCESSOR_H_ */
