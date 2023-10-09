#pragma once

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

class CsvRow
{
private:
  std::string         line_;
  std::vector<int>    data_;

public:
  inline std::size_t size() const { return data_.size() - 1; }

  std::string_view operator[](std::size_t index) const
  {
    return std::string_view(&line_[data_[index] + 1], data_[index + 1] -  (data_[index] + 1));
  }

  void read(std::istream & str)
  {
    do {
      std::getline(str, line_);
    } while (line_[0] == '#');

    data_.clear();
    data_.emplace_back(-1);
    std::string::size_type pos = 0;

    line_.erase(
      std::remove_if(
        line_.begin(),
        line_.end(),
        isSpace()),
      line_.end());

    while((pos = line_.find(',', pos)) != std::string::npos)
    {
      data_.emplace_back(pos);
      ++pos;
    }

    pos   = line_.size();
    data_.emplace_back(pos);
  }
};

std::istream & operator>>(std::istream & str, CsvRow & data)
{
  data.read(str);
  return str;
}

class CsvReader
{
private:
  std::ifstream file_;
  CsvRow row_;

public:
  CsvReader(const std::string & fn)
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

  bool eof() { return file_.eof(); }

  auto next() -> std::vector<std::string>
  {
    std::vector<std::string> words;
    file_ >> row_;

    for (size_t i = 0; i < row_.size(); i++)
    {
      words.push_back(std::string(row_[i]));
    }

    return words;
  }
};
