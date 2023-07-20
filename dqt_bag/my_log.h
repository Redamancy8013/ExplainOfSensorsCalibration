#ifndef MY_LOG_H
#define MY_LOG_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdint.h>
#include <stdlib.h>

using namespace std;

typedef enum log_type {
	WARNING, ERROR, FATAL, DEBUG, INFO,
} log_type_cur;

void MyLogInit(const string log_file_name);

class MyLog {

public:

	MyLog(log_type_cur type) :
			current_type(type) {
	}
	;
	~MyLog();

	friend void MyLogInit(const string log_file_name);
	std::ostream& record(log_type_cur type, const int line,
			const std::string& function, const std::string& file);
	const char * ConvertEnumToString(log_type_cur type_data);

private:

	string log_file_name_;
	static std::ofstream log_file_;
	log_type_cur current_type;
};

#define LOG(log_type)   \
MyLog(log_type).record(log_type, __LINE__,__FUNCTION__, __FILE__)

#endif //MY_LOG_H
