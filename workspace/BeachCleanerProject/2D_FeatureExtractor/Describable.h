
#ifndef DESCRIBABLE_H_
#define DESCRIBABLE_H_

#include <string>
using namespace std;

class Describable {
public:
	Describable();
	virtual ~Describable();
	string GetCurrentSetting() ;
	virtual string GetClassName() = 0;
	virtual string GetComments() = 0;
	virtual string GetCurrentParams() = 0;
};

#endif /* DESCRIBABLE_H_ */
