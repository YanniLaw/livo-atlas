#ifndef LIVO_ATLAS_CORE_PARAMETER_READER_H_
#define LIVO_ATLAS_CORE_PARAMETER_READER_H_

#include <string>
#include <vector>

namespace livo_atlas {

// Native adapters provide typed reads; the application schema uses dotted keys.
class ParameterReader {
 public:
  virtual ~ParameterReader() = default;
  virtual bool Boolean(const std::string& name, bool fallback) = 0;
  virtual int Integer(const std::string& name, int fallback) = 0;
  virtual double Number(const std::string& name, double fallback) = 0;
  virtual std::string String(const std::string& name,
                             const std::string& fallback) = 0;
  virtual std::vector<double> NumberArray(const std::string& name) = 0;
  virtual std::vector<int> IntegerArray(const std::string& name,
                                        const std::vector<int>& fallback) = 0;
  virtual std::string Name(const std::string& name) const = 0;
};

}  // namespace livo_atlas
#endif  // LIVO_ATLAS_CORE_PARAMETER_READER_H_
