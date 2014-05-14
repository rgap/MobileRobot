
#ifndef TEXTURE_ANALYZER_FACTORIES_H_
#define TEXTURE_ANALYZER_FACTORIES_H_

#include "typedefs.h"

class TextureAnalyzerFactories {
public:

	TextureAnalyzerFactories();
	virtual ~TextureAnalyzerFactories();

	static ColorCooMatrixTextureAnalyzerPtr ColorEneConCorrTextureAnalyzer(int distance_relation, int max_pixels_value, int num_windows_height,
			int num_windows_width);
	static GrayCooMatrixTextureAnalyzerPtr GrayEneConCorrTextureAnalyzer(int distance_relation, int max_pixels_value, int num_windows_height,
			int num_windows_width);
};

#endif /* TEXTURE_ANALYZER_FACTORIES_H_ */
