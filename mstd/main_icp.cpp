#include <CsvReader.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <fmt/format.h>
#include <fmt/printf.h>

#include <fstream>
#include <string>

DEFINE_string(sequence_dir, "", "");
DEFINE_string(method, "icp", "'icp' or 'gicp'");

void displayPCDHeader(const std::string& pcd_file_path)
{
  std::ifstream pcd_file(pcd_file_path, std::ios::in);
  if (!pcd_file.is_open())
  {
    LOG(ERROR) << fmt::format(
      "Failed to open pcd file {}",
      pcd_file_path);
    return;
  }

  std::string line;
  while (std::getline(pcd_file, line))
  {
    if (line == "DATA ascii") {
        // Reached the end of the header
        break;
    }
    LOG(INFO) << line;
  }

  pcd_file.close();
}

void readLidarData(
  const std::string & data_fn,
  std::vector<double> out_timestamps,
  std::vector<std::string> out_scan_paths)
{
  CsvReader reader(data_fn);

  while (true)
  {
    auto words = reader.next();
    if (words.empty()) { break; }

    LOG(INFO) << words[0];
    double timestamp = std::stod(words[0]);
    std::string scan_path = words[1];
    // fmt::print("timestamp: {}, scan_path: {}\n", timestamp, scan_path);
    out_timestamps.push_back(timestamp);
    out_scan_paths.push_back(scan_path);
  }

  LOG(INFO) << out_timestamps.size();
  LOG(INFO) << out_scan_paths.size();
}

int main (int argc, char * argv[])
{
  google::SetVersionString("1.0.0");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  // google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  auto data_fn = fmt::format("{}/data.csv", FLAGS_sequence_dir);
  std::vector<double> timestamps;
  std::vector<std::string> scan_paths;
  readLidarData(data_fn, timestamps, scan_paths);

  for (size_t i = 0; i < timestamps.size(); i++)
  {
    LOG(INFO) << fmt::format("t: {:.3f}, path: {}", timestamps[i], scan_paths[i]);
  }

  return 0;
}
