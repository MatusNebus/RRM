#ifndef BLOCK1_NEBUS__POINT_FILE_WRITER_HPP_
#define BLOCK1_NEBUS__POINT_FILE_WRITER_HPP_

#include <fstream>
#include <string>
#include <vector>

class PointFileWriter
{
public:
  explicit PointFileWriter(const std::string & filename)
  : filename_(filename)
  {
  }

  bool savePoint(int id, const std::vector<double> & positions, double velocity)
  {
    if (positions.size() != 3) {
      return false;
    }

    std::ofstream file(filename_, std::ios::app);
    if (!file.is_open()) {
      return false;
    }

    file << id << " "
         << positions[0] << " "
         << positions[1] << " "
         << positions[2] << " "
         << velocity << "\n";

    return true;
  }

private:
  std::string filename_;
};

#endif
