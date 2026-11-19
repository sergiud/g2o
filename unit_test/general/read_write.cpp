// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <new>
#include <ostream>
#include <ranges>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "g2o/core/factory.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/parameter.h"
#include "g2o/types/data/raw_laser.h"
#include "g2o/types/data/types_data.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sclam2d/types_sclam2d.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d_addons/types_slam2d_addons.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "gtest/gtest.h"

G2O_USE_TYPE_GROUP(data)
G2O_USE_TYPE_GROUP(expmap)
G2O_USE_TYPE_GROUP(icp)
G2O_USE_TYPE_GROUP(sba)
G2O_USE_TYPE_GROUP(sclam)
G2O_USE_TYPE_GROUP(sim3)
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(slam2d_segment)
G2O_USE_TYPE_GROUP(slam3d)
G2O_USE_TYPE_GROUP(slam3d_addons)

namespace {

//! Element of a graph file reduced to the two functions of its text form.
class TextForm {
 public:
  explicit TextForm(std::unique_ptr<g2o::HyperGraph::HyperGraphElement> element)
      : _element(std::move(element)) {}

  bool valid() const { return _element != nullptr; }

  bool read(std::istream& is) {
    if (auto* v =
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(_element.get())) {
      return v->read(is);
    }
    if (auto* e = dynamic_cast<g2o::OptimizableGraph::Edge*>(_element.get())) {
      // An edge with any number of vertices ends their list with a "||",
      // which the graph consumes before the edge reads the rest.
      const std::istream::pos_type start = is.tellg();
      std::string separator;
      if (!(is >> separator) || separator != "||") {
        is.clear();
        is.seekg(start);
      }
      return e->read(is);
    }
    if (auto* p = dynamic_cast<g2o::Parameter*>(_element.get())) {
      return p->read(is);
    }
    return dynamic_cast<g2o::HyperGraph::Data&>(*_element).read(is);
  }

  bool write(std::ostream& os) const {
    if (auto* v =
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(_element.get())) {
      return v->write(os);
    }
    if (auto* e = dynamic_cast<g2o::OptimizableGraph::Edge*>(_element.get())) {
      return e->write(os);
    }
    if (auto* p = dynamic_cast<g2o::Parameter*>(_element.get())) {
      return p->write(os);
    }
    return dynamic_cast<g2o::HyperGraph::Data&>(*_element).write(os);
  }

 private:
  std::unique_ptr<g2o::HyperGraph::HyperGraphElement> _element;
};

//! Type of a graph element with the function that constructs it.
struct ElementType {
  std::string name;
  std::function<std::unique_ptr<g2o::HyperGraph::HyperGraphElement>()>
      construct;
};

void PrintTo(const ElementType& type, std::ostream* os) { *os << type.name; }

template <typename T>
ElementType unregistered(std::string name) {
  return {std::move(name), [] { return std::make_unique<T>(); }};
}

/**
 * Types of the elements that appear in graph files: the registered ones and
 * those that applications write with their own tags.
 */
std::vector<ElementType> elementTypes() {
  std::vector<std::string> tags;
  g2o::Factory::instance()->fillKnownTypes(tags);
  std::vector<ElementType> types;
  for (const std::string& tag : tags) {
    int type = -1;
    g2o::Factory::instance()->knowsTag(tag, &type);
    // Caches are registered to construct them, but a file never holds them.
    if (type != g2o::HyperGraph::HGET_CACHE) {
      types.push_back(
          {tag, [tag] { return g2o::Factory::instance()->construct(tag); }});
    }
  }
  types.push_back(unregistered<g2o::EdgeInverseSim3ProjectXYZ>(
      "EdgeInverseSim3ProjectXYZ"));
  types.push_back(unregistered<g2o::EdgeProjectPSI2UV>("EdgeProjectPSI2UV"));
  types.push_back(unregistered<g2o::EdgeSE3ProjectXYZ>("EdgeSE3ProjectXYZ"));
  types.push_back(unregistered<g2o::EdgeSE3ProjectXYZOnlyPose>(
      "EdgeSE3ProjectXYZOnlyPose"));
  types.push_back(
      unregistered<g2o::EdgeStereoSE3ProjectXYZ>("EdgeStereoSE3ProjectXYZ"));
  types.push_back(unregistered<g2o::EdgeStereoSE3ProjectXYZOnlyPose>(
      "EdgeStereoSE3ProjectXYZOnlyPose"));
  types.push_back(unregistered<g2o::RawLaser>("RawLaser"));
  return types;
}

class TypeTextForm : public ::testing::TestWithParam<ElementType> {};

}  // namespace

TEST_P(TypeTextForm, ReadsWhatItWrites) {
  const TextForm written(GetParam().construct());
  ASSERT_TRUE(written.valid());
  std::ostringstream text;
  ASSERT_TRUE(written.write(text));

  // A line written by hand ends with its last value.
  std::string line = text.str();
  line.erase(line.find_last_not_of(" \t") + 1);
  TextForm read(GetParam().construct());
  std::istringstream is(line);
  EXPECT_TRUE(read.read(is)) << line;
}

TEST_P(TypeTextForm, RejectsATruncatedLine) {
  const TextForm written(GetParam().construct());
  ASSERT_TRUE(written.valid());
  std::ostringstream text;
  ASSERT_TRUE(written.write(text));
  std::istringstream tokens(text.str());
  std::vector<std::string> values{std::istream_iterator<std::string>{tokens},
                                  std::istream_iterator<std::string>{}};
  ASSERT_FALSE(values.empty());
  values.pop_back();
  std::string truncated;
  for (const std::string& value : values) {
    truncated += value + ' ';
  }

  TextForm read(GetParam().construct());
  std::istringstream is(truncated);
  EXPECT_FALSE(read.read(is)) << truncated;
}

INSTANTIATE_TEST_SUITE_P(Registered, TypeTextForm,
                         ::testing::ValuesIn(elementTypes()),
                         [](const ::testing::TestParamInfo<ElementType>& info) {
                           std::string name = info.param.name;
                           for (char& c : name) {
                             if (!std::isalnum(static_cast<unsigned char>(c))) {
                               c = '_';
                             }
                           }
                           return name;
                         });

namespace {

/**
 * Constructs an element in memory whose bytes all have the given value, so
 * that a member its constructor leaves uninitialized keeps that value.
 */
template <typename T>
class ConstructedOver {
 public:
  explicit ConstructedOver(std::byte value) {
    std::ranges::fill(_storage, value);
    _element = new (_storage.data()) T;
  }
  ~ConstructedOver() { _element->~T(); }

  ConstructedOver(const ConstructedOver&) = delete;
  ConstructedOver& operator=(const ConstructedOver&) = delete;

  const T& operator*() const { return *_element; }

 private:
  alignas(T) std::array<std::byte, sizeof(T)> _storage;
  T* _element;
};

//! Bytes of a quiet NaN, which every IEEE 754 double of all ones is.
constexpr std::byte AllOnes{0xFF};

}  // namespace

/**
 * A default edge whose measurement is a vector measures zero, so that writing
 * it writes numbers a graph file can hold.
 */
TEST(DefaultEdge, MeasuresZero) {
  const ConstructedOver<g2o::EdgeProjectPSI2UV> projection(AllOnes);
  const ConstructedOver<g2o::EdgeSE2PointXY> landmark(AllOnes);

  EXPECT_TRUE((*projection).measurement().isZero(0.));
  EXPECT_TRUE((*landmark).measurement().isZero(0.));
}

namespace {

//! Line whose count of values the type reads before the values.
struct CountedLine {
  std::string type;
  std::string prefix;
  std::string suffix;
};

void PrintTo(const CountedLine& line, std::ostream* os) { *os << line.type; }

class CountedValues : public ::testing::TestWithParam<CountedLine> {
 protected:
  bool readWithCount(const std::string& count) const {
    TextForm element(g2o::Factory::instance()->construct(GetParam().type));
    std::istringstream is(GetParam().prefix + count + GetParam().suffix);
    return element.read(is);
  }
};

}  // namespace

//! A negative count cannot size the values that follow it.
TEST_P(CountedValues, RejectsANegativeCount) {
  EXPECT_FALSE(readWithCount("-1"));
}

/**
 * A count larger than the values of the line fails when the line ends,
 * rather than allocating the memory for every value it announces.
 */
TEST_P(CountedValues, RejectsACountBeyondTheLine) {
  EXPECT_FALSE(readWithCount("2000000000"));
}

INSTANTIATE_TEST_SUITE_P(
    Registered, CountedValues,
    ::testing::Values(CountedLine{"ROBOTLASER1",
                                  "0 -1.57 3.14 0.0174 50 0.1 0 ", " 1 2 3"},
                      CountedLine{"EDGE_SE2_LOTSOFXY", "|| ", " 1 2 3"},
                      CountedLine{"EDGE_SE3_LOTSOF_XYZ", "|| ", " 1 2 3"}),
    [](const ::testing::TestParamInfo<CountedLine>& info) {
      std::string name = info.param.type;
      std::ranges::replace_if(
          name,
          [](char c) { return !std::isalnum(static_cast<unsigned char>(c)); },
          '_');
      return name;
    });
