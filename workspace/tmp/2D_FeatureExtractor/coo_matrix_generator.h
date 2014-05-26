#ifndef COO_MATRIX_GENERATOR_H_
#define COO_MATRIX_GENERATOR_H_

#include "Describable.h"
#include <vector>

using namespace std;

namespace cv {
class Mat;
}

/**
 * @enum GLCM_RELATION List all possible relations of length two between three channels,
 * the last one (GRAY) is an exception, because is just a conceptual relation.
 */

enum GLCM_RELATION {
	RGB_RR = (1 << 0) + (1 << 3),
	RGB_RG = (1 << 0) + (1 << 4),
	RGB_RB = (1 << 0) + (1 << 5),
	RGB_GG = (1 << 1) + (1 << 4),
	RGB_GR = (1 << 1) + (1 << 3),
	RGB_GB = (1 << 1) + (1 << 5),
	RGB_BR = (1 << 2) + (1 << 3),
	RGB_BG = (1 << 2) + (1 << 4),
	RGB_BB = (1 << 2) + (1 << 5),
	GRAY = 0
};

class AbstractCooMatrixGenerator: public Describable {
public:

	AbstractCooMatrixGenerator();
	AbstractCooMatrixGenerator(vector<pair<int, int> > offset, bool symmetric,
			int out_levels);
	virtual ~AbstractCooMatrixGenerator();
	virtual void CalculeCooMatrix(cv::Mat *image_input, cv::Mat *output_glcm,
			int in_num_levels, GLCM_RELATION relation) = 0;

	virtual void SetOffset(vector<pair<int, int> > offset);
	virtual void SetSymmetricFlag(bool symmetric);
	virtual vector<pair<int, int> > GetOffset();
	virtual void SetBannedValues(vector<int> banned_gray_values);
	virtual vector<int> GetBannedValues();
	virtual void SetOutLevels(int out_levels);

	virtual int GetOutLevels();

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();

protected:
	enum {
		NUM_OF_LEVELS = 256
	};
	vector<pair<int, int> > offset_; ///< Vector of 2d-distances, each element specifying the distance between the pixel of interest and its neighbors, by default it contains one element: \b [ \b [\b 1, \b 0] \b ]
	bool * banned_values_; ///<  Array of size NUM_OF_LEVELS, whose elements are true only if they are banned pixel values and false otherwise.  By default all elements in the array are \b false.
	bool symmetric_; ///< It specifies if the output co-occurrence matrix will be symmetric or not, by default this parameter is \b true.
	int out_levels_; ///< It specifies if the number of rows and columns the co-ocurrence matrix will going to have. By default this parameter is equal to \b 16
};

class CooMatrixGeneratorFrom3CImage: public AbstractCooMatrixGenerator {
public:

	CooMatrixGeneratorFrom3CImage();
	CooMatrixGeneratorFrom3CImage(vector<pair<int, int> > offset,
			bool symmetric, int out_levels);

	virtual ~CooMatrixGeneratorFrom3CImage();
	virtual void CalculeCooMatrix(cv::Mat *image_input, cv::Mat *output_glcm,
			int in_num_levels, GLCM_RELATION relation);

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();
private:

	void QuantizeMatrix(cv::Mat* image_input_output,
			const int& current_num_levels, const int& desired_num_levels);
};

class CooMatrixGeneratorFrom1CImage: public AbstractCooMatrixGenerator {
public:

	CooMatrixGeneratorFrom1CImage();
	CooMatrixGeneratorFrom1CImage(vector<pair<int, int> > offset,
			bool symmetric, int out_levels);

	virtual ~CooMatrixGeneratorFrom1CImage();

	virtual void CalculeCooMatrix(cv::Mat *image_input, cv::Mat *output_glcm,
			int in_num_levels, GLCM_RELATION relation = GRAY);

	virtual string GetClassName();
	virtual string GetComments();
	virtual string GetCurrentParams();
private:

	void QuantizeMatrix(cv::Mat* image_input_output,
			const int& current_num_levels, const int& desired_num_levels);
};

#endif /* COO_MATRIX_GENERATOR_H_ */
