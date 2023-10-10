#ifndef CSV_READER_HPP__
#define CSV_READER_HPP__

#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <glog/logging.h>
// https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c#comment11041441_1120224

struct isSpace
{
  bool operator()(unsigned c)
  {
    return (c == ' ' || c == '\n' || c == '\r' ||
            c == '\t' || c == '\v' || c == '\f');
  }
};

class CsvReader
{
private:
  std::string fn_;
  std::ifstream file_;
  int n_cols_;

public:
  CsvReader()=delete;
  CsvReader(
    const std::string & fn,
    const int & n_cols)
    : fn_(fn),
      n_cols_(n_cols)
  {
    file_.open(fn);
    if (!file_.is_open()) {
      LOG(ERROR) << "Mask file is not opened! : " << fn;
    }
  }

  ~CsvReader()
  {
    if (file_.is_open()) {
      file_.close();
    }
  }

  bool isOpened() { return file_.is_open(); }

  bool eof() { return file_.eof(); }

  auto next() -> std::vector<std::string>
  {
    std::string line;
    std::vector<std::string> words;

    if (std::getline(file_, line))
    {
      line.erase(
        std::remove_if(
          line.begin(),
          line.end(),
          isSpace()),
        line.end());
      std::istringstream ss(line);
      std::string token;
      while (std::getline(ss, token, ','))
      {
        words.push_back(token);
      }

      if (line.empty() || words.size() != n_cols_)
      {
        return std::vector<std::string>();
      }
    }

    return words;
  }
};

#endif
