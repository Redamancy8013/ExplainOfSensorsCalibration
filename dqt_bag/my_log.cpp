#include "my_log.h"
#include <cstdlib>
#include <ctime>

const char* TypeName[] = { "WARNING", "ERROR", "FATAL", "DEBUG", "INFO" };

std::ostream& MyLog::record(log_type_cur type, const int line,
		const std::string& function, const std::string& file) {
	time_t tm;
	time(&tm);
	char time_string[128];
	ctime_r(&tm, time_string);
	int i = type;
	return log_file_ << "[" << TypeName[i] << "]" << " " << time_string
			<< "File (" << file << ")" << " Function (" << function << ")"
			<< " " << "Line (" << line << ")" << std::flush;
}

std::ofstream MyLog::log_file_;

void MyLogInit(const string log_file_name) {

	MyLog::log_file_.open(log_file_name);
}

MyLog::~MyLog() {

	if (FATAL == current_type) {
		log_file_.close();
		abort();
	}

}

