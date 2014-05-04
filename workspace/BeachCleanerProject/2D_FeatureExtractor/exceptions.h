
#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

#include <exception>

using namespace std;

class ExDirNoFound : public std::exception{
	virtual const char* what() const throw(){
		return "Directory No Found";
	}
} ;

class ExFileNameNoFound : public std::exception{
	virtual const char * what() const throw(){
		return "File Name No Found";
	}
};

class ExBadFileFormatDetected : public std::exception{
	virtual const char * what() const throw(){
		return "File bad formatted";
	}
};

class ExBadFormat : public std::exception{
	virtual const char * what () const throw(){
		return "Bad Format of the pattern";
	}
}  ;

class ExBadDirCreation : public std::exception{
	virtual const char * what () const throw(){
		return "Directory can't be created, check access permissions or correctness of the path provided";
	}
} ;

class ExMismatchParams : public std::exception{
	virtual const char * what () const throw(){
		return "Mismatch in the parameters";
	}
};

class ExBadType : public std::exception{
	virtual const char * what() const throw(){
		return "Bad type entered, see the method documentation";
	}
};

class ExFileNotSupported : public std::exception{
	virtual const char* what() const throw(){
		return "File not supported";
	}
} ;

class ExIndexLimitExceeded : public std::exception{
	virtual const char* what() const throw(){
		return "Index limit exceeded";
	}
};


#endif /* EXCEPTIONS_H_ */
