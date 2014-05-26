
#include "Describable.h"
#include <sstream>

Describable::Describable() {
	// TODO Auto-generated constructor stub

}

Describable::~Describable() {
	// TODO Auto-generated destructor stub
}

string Describable::GetCurrentSetting() {
	stringstream streamer;
	streamer << endl << "****** Class: " << GetClassName() << " {" << endl << endl;
	streamer << " --- Comments: " << GetComments() << endl << endl;
	streamer << " --- Setting Parameters: " << endl;
	streamer << GetCurrentParams() << endl;
	streamer << "}" << endl;
	return streamer.str();
}


