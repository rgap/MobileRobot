/*
 * feature_formatter.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: rayner
 */

#include <fstream>
#include <map>
#include <iostream>
#include <sstream>
#include <set>
#include "exceptions.h"
#include "file_format_manager.h"

FeaturesFormatManager::FeaturesFormatManager() {
}

FeaturesFormatManager::~FeaturesFormatManager() {
}

int FeaturesFormatManager::GetNumberOfFeatures() {
	return number_of_features_;
}

int FeaturesFormatManager::GetNumberOfInputs() {
	return number_of_inputs_;
}

void FeaturesFormatManager::SetNumberOfFeatures(int number_of_features) {
	number_of_features_ = number_of_features;
}

void FeaturesFormatManager::SetNumberOfInputs(int number_of_inputs) {
	number_of_inputs_ = number_of_inputs;
}


//////////////////////////////////// Formatter ///////////////////////////////
PredefinedFormatManager::PredefinedFormatManager() {
}

PredefinedFormatManager::~PredefinedFormatManager() {
}

void PredefinedFormatManager::Print(const string &output_file_name,
		const vector<string> &input_names, const vector<string> &input_classes,
		const cv::Mat & input_features, int mode) throw (exception) {
	int number_of_inputs = input_names.size();
	if (input_features.type() != CV_32FC1)
		throw ExBadType();
	if (number_of_inputs != (int) input_classes.size())
		throw ExMismatchParams();
	if (number_of_inputs != input_features.rows)
		throw ExMismatchParams();
	FeaturesFormatManager::SetNumberOfInputs(number_of_inputs);
	FeaturesFormatManager::SetNumberOfFeatures(input_features.cols);
	ofstream xout;
	if (mode > 0)
		xout.open(output_file_name.c_str(), ios::app);
	else
		xout.open(output_file_name.c_str());
	if (xout.fail())
		throw ExFileNameNoFound();
	for (int inp_ind = 0; inp_ind < number_of_inputs; inp_ind++) {
		xout << input_names[inp_ind] << " " << input_classes[inp_ind];
		const float * row_feature = input_features.ptr<float> (inp_ind);
		for (int fea_ind = 0; fea_ind < input_features.cols; fea_ind++) {
			xout << " " << row_feature[fea_ind];
		}
		xout << endl;
	}
	xout.close();
	GenerateConfFile(output_file_name, output_file_name + ".conf");
}

void PredefinedFormatManager::ExtractNumInputsAndFeatures(
		const string file_name, vector<string> *distinct_classes)
		throw (exception) {
	int number_of_inputs = 0;
	int number_of_features = 0;
	int number_of_distinct_classes = 0;
	ifstream xin;
	xin.open((file_name + ".conf").c_str());
	if (xin.fail()) {
		GenerateConfFile(file_name, file_name + ".conf");
		xin.open((file_name + ".conf").c_str());
	}
	string class_name;
	xin >> number_of_inputs >> number_of_features >> number_of_distinct_classes;
	for (int pos_class = 0; pos_class < number_of_distinct_classes; ++pos_class) {
		xin >> class_name;
		distinct_classes->push_back(class_name);
	}
	xin.close();
	FeaturesFormatManager::SetNumberOfFeatures(number_of_features);
	FeaturesFormatManager::SetNumberOfInputs(number_of_inputs);
}

void PredefinedFormatManager::Retrieve(const string &input_file_name,
		vector<string> *input_names, vector<string> *input_classes,
		cv::Mat *input_features, vector<string> *distinct_classes)
		throw (exception) {
	ExtractNumInputsAndFeatures(input_file_name, distinct_classes);
	int number_of_inputs = FeaturesFormatManager::GetNumberOfInputs();
	int number_of_features = FeaturesFormatManager::GetNumberOfFeatures();

	input_names->resize(number_of_inputs);
	input_classes->resize(number_of_inputs);
	//cout << number_of_inputs << " " << number_of_features << endl;
	(*input_features) = cv::Mat(number_of_inputs, number_of_features, CV_32FC1);
	ifstream xin(input_file_name.c_str());
	string temp;
	if (xin.fail())
		throw ExFileNameNoFound();

	for (int inp_ind = 0; inp_ind < number_of_inputs; inp_ind++) {
		xin >> (*input_names)[inp_ind] >> (*input_classes)[inp_ind];
		float * row_feature = input_features->ptr<float> (inp_ind);
		for (int fea_ind = 0; fea_ind < number_of_features; fea_ind++) {
			if (xin.eof())
				throw ExBadFileFormatDetected();
			xin >> row_feature[fea_ind];
		}
	}

}

void PredefinedFormatManager::GenerateConfFile(
		const string& training_feature_vec, const string& conf_file_name)
		throw (exception) {
	ifstream xin(training_feature_vec.c_str());
	if (xin.fail())
		throw ExFileNameNoFound();
	string line, clase;
	set < string > classes;
	int number_of_features = 0;
	int number_of_inputs = 0;
	double feature;
	if (xin.eof())
		throw ExBadFileFormatDetected();
	getline(xin, line);
	stringstream parser;
	parser << line;
	parser >> clase >> clase;
	while (parser >> feature) {
		number_of_features++;
	}
	number_of_inputs = 1;
	while (getline(xin, line)) {
		number_of_inputs++;
		stringstream tokenizer;
		tokenizer << line;
		tokenizer >> clase >> clase;
		classes.insert(clase);
	}
	xin.close();
	ofstream xout(conf_file_name.c_str());
	xout << number_of_inputs << " " << number_of_features << " "
			<< classes.size() << endl;
	set<string>::iterator p;
	for (p = classes.begin(); p != classes.end(); p++) {
		xout << *p << endl;
	}
	xout.close();
}


////////////////////////////////////// WekaFormatManager class ////////////////////////////////
WekaFormatManager::WekaFormatManager() {
}

WekaFormatManager::~WekaFormatManager() {
}

void WekaFormatManager::Print(const std::string& output_file_name,
		const std::vector<std::string>& input_names,
		const std::vector<std::string>& input_classes,
		const cv::Mat& input_features, int mode) throw(std::exception){
	if(input_names.size() != input_classes.size()) throw ExMismatchParams();
	if((int)input_names.size() != input_features.rows ) throw ExMismatchParams();
	std::ofstream xout;
	if(mode > 0) xout.open(output_file_name.c_str(), std::ios::app);
	else xout.open((output_file_name + vector_extension()).c_str());
	xout << "@relation vectores" << std::endl;

	std::set<std::string> distinct_classes;
	for(int ind = 0; ind < (int)input_names.size() ; ++ind){
		distinct_classes.insert(input_classes[ind]);
		if(input_classes[ind] == "") std::cout << ind << " asdfasfasdfas" << input_classes[ind] << std::endl;
	}
	for(int ind = 0; ind < input_features.cols; ++ind){
		std::stringstream stream;
		stream << ind ;
		xout << "@attribute Feature" << stream.str() << " numeric" << std::endl;
	}

	xout << "@attribute class {";
	std::set<std::string>::iterator p = distinct_classes.begin();
	while(p != distinct_classes.end()){
		if(p != distinct_classes.begin()) xout << ",";
		//std::cout << distinct_classes.size() << *p << std::endl;
		xout << *p;
		p++;
	}
	xout << "}" << std::endl;
	xout << "@data" << std::endl;

	for(int input_pos = 0; input_pos < input_features.rows; ++input_pos){
		const float * row_features = input_features.ptr<float>(input_pos);
		for(int input_fea = 0; input_fea < input_features.cols ; ++ input_fea){
			xout << row_features[input_fea] << ",";
		}
		xout << input_classes[input_pos] << std::endl;
	}

	xout.close();
}



void WekaFormatManager::Retrieve(const std::string& input_file_name,
		std::vector<std::string>* input_names,
		std::vector<std::string>* input_classes, cv::Mat* input_features,
		std::vector<std::string>* distinct_classes) throw(std::exception){

	std::size_t pos_point = input_file_name.rfind(".");

	if(pos_point == std::string::npos || input_file_name.substr(pos_point) != vector_extension()){
		throw ExFileNotSupported();
	}
	distinct_classes->clear();
	input_classes->clear();
	input_names->clear();
	std::ifstream xin;
	xin.open(input_file_name.c_str());
	if(xin.fail()) throw ExDirNoFound();
	std::string line, token;
	std::string name_att = "@attribute";
	std::string name_rel = "@relation";
	std::string name_dat = "@data";
	std::string name_class = "class";
	bool reach_data = false;
	int number_of_features = 0, number_of_inputs = 0;
	while(!xin.eof()){
		getline(xin, line);
		if((int)line.size() == 0) continue;
		std::istringstream stream_line(line);
		if(reach_data){
			number_of_inputs ++;
		}else{
			stream_line >> token;
			if(token[0] == '%') continue;
			ToLowerCase(&token);
			if(token == name_rel){
				continue;
			}else if(token == name_att){
				stream_line >> token;
				if(token == name_class){
					getline(stream_line, line);
					std::string category = "";
					for( int pos = 0; pos < (int)line.size() ; ++pos){
						if( !( line[pos] == ' ' || line[pos] == '{'  || line[pos] == '\t') ){
							if(line[pos] == ','|| line[pos] == '}'){
								//std::cout << distinct_classes->size() << " " << category << std::endl;
								distinct_classes->push_back(category);
								category = "";
							}else{
								category += line[pos];
							}
						}
					}
				}else number_of_features ++;
				continue;
			}else if(token == name_dat){

				reach_data = true;
				continue;
			}
		}
	}
	//std::cout << number_of_inputs << " " << number_of_features << std::endl;
	xin.close();
	(*input_features) = cv::Mat(number_of_inputs, number_of_features, CV_32FC1);
	reach_data  = false;

	xin.open(input_file_name.c_str());
	input_names->resize(number_of_inputs);
	input_classes->resize(number_of_inputs);
	std::string number;
	int cont = 0;
	float feature;
	while(!xin.eof()){
		getline(xin, line);
		if((int)line.size() == 0) continue;
		//if(cont == 2848 ) std::cout << cont << " " << line << std::endl;
		if(reach_data){
			//std::cout << line << std::endl;
			int num_fea = 0;
			number = "";
			float * row_input = input_features->ptr<float>(cont);
			for(int pos = 0; pos < (int)line.size(); ++pos){
				if(line[pos] == ' ') continue;
				if(line[pos] == ','){
					std::stringstream stream_fea ;
					stream_fea << number;
					if(!(stream_fea >> feature)) std::cout<< "failed: " << number << std::endl;
					row_input[num_fea] = feature;
					num_fea ++;
					number = "";
				}else{
					number += line[pos];
				}
			}
			(*input_classes)[cont] = number;
			//if(number.size() == 0) std::cout << "endtre" << std::endl;
			cont ++;
		}else{
			std::istringstream stream_line(line);
			stream_line >> token;
			ToLowerCase(&token);
			if(token == name_dat) reach_data = true;
		}
	}
	FeaturesFormatManager::SetNumberOfInputs(number_of_inputs);
	FeaturesFormatManager::SetNumberOfFeatures(number_of_features);
	xin.close();
}

std::string WekaFormatManager::vector_extension() {
	return ".arff";
}

void WekaFormatManager::ToLowerCase(std::string* token) {
	//std::cout << (*token) << std::endl;
	for(int ind =  0 ; ind < (int)token->size() ; ++ind){
		if((*token)[ind] >= 'A' && (*token)[ind] <= 'Z'){
			(*token)[ind] = ((*token)[ind] - 'A') + 'a';
		}
	}
	//std::cout << (*token) << std::endl;
}
